// DeFT, record-file variant using hybrid-TISE: no custom `deft`
// instrumentation required.
//
// record_main.cc replays a recorded Planning trial by reading the `deft`
// block that an instrumented Apollo build embeds in every /apollo/planning
// message (see modules/planning/proto/planning.proto). That block names the
// frame's construction time and the header sequence_num of every input that
// fed the cycle, so replay is a pure lookup.
//
// This variant reconstructs the same information from fields stock Apollo
// already publishes, so it can replay records produced by an UNPATCHED
// Apollo. It is the C++ analogue of the Python `DeFTApollo` variant --
// hybrid-TISE: metadata where Apollo publishes it, Time-Sensitive Input
// Search where it does not.
//
// Per planning cycle:
//
//   frame time t_F
//       t_F = header.timestamp_sec
//             + trajectory_point(0).relative_time
//             - (is_replan ? kReplanInitOffsetSec : 0.0)
//
//     Apollo projects the init point one planning cycle ahead when it
//     replans, and stitches from the previous trajectory when it does not.
//     on_lane_planning.cc's Plan() copies init_point from
//     stitching_trajectory.back() only under FLAGS_enable_record_debug,
//     which is why debug.planning_data.init_point is absent on non-replan
//     cycles -- `is_replan` is the field that is always present.
//
//   routing / chassis / localization
//     debug.planning_data embeds the COMPLETE RoutingResponse, Chassis and
//     LocalizationEstimate (adc_position) that fed the cycle, not merely
//     their sequence numbers. They are used directly, so these three inputs
//     need no record index and cannot go missing.
//
//   prediction
//     debug.planning_data.prediction_header carries the Header only, so the
//     PredictionObstacles body is looked up by sequence_num in the record.
//
//   traffic_light
//     Not present in debug.planning_data at all. Recovered by TISE: the
//     newest traffic light message recorded at or before t_F, with a
//     non-decreasing sequence number across frames (Heuristic 3). Note that
//     a cycle may legitimately consume a message published before the
//     routing response, so the index below is built over the whole record
//     with no start gate.
//
//   stories / pad
//     Neither is published in debug.planning_data and neither can be
//     recovered; they are left unset, exactly as record_main.cc does when
//     the corresponding deft header is absent.
//
// Everything downstream -- the single continuous PlanningBase instance, the
// mocked Clock, the PlanningBase::RunOnce() entry point, the output layout
// -- is identical to record_main.cc, so the two can be diffed frame by
// frame.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/time/clock.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/planning_base.h"

using ::apollo::cyber::common::EnsureDirectory;
using ::apollo::cyber::common::GetProtoFromFile;
using ::apollo::cyber::common::SetProtoToBinaryFile;
using ::apollo::cyber::record::RecordMessage;
using ::apollo::cyber::record::RecordReader;

using ::apollo::canbus::Chassis;
using ::apollo::localization::LocalizationEstimate;
using ::apollo::perception::TrafficLightDetection;
using ::apollo::prediction::PredictionObstacles;
using ::apollo::routing::RoutingResponse;

using ::apollo::planning::ADCTrajectory;
using ::apollo::planning::DependencyInjector;
using ::apollo::planning::LocalView;
using ::apollo::planning::OnLanePlanning;
using ::apollo::planning::PlanningBase;
using ::apollo::planning::PlanningConfig;

namespace {

// Apollo projects the planning init point one cycle (0.1 s at the default
// 10 Hz planning rate) ahead of the frame construction time when it
// replans. Validated against the instrumented `deft` ground truth on 632
// planning cycles across three records: exact for every frame.
constexpr double kReplanInitOffsetSec = 0.1;

constexpr double kNanosPerSecond = 1e9;

// header.sequence_num() -> serialized proto bytes, for one topic.
using SeqIndex = std::unordered_map<int32_t, std::string>;

template <typename T>
void IndexBySequence(const std::string& content, SeqIndex* index) {
  T msg;
  if (msg.ParseFromString(content)) {
    (*index)[msg.header().sequence_num()] = content;
  }
}

template <typename T>
bool LookupBySequence(const SeqIndex& index, int32_t seq, T* out) {
  if (seq < 0) {
    return false;
  }
  auto it = index.find(seq);
  if (it == index.end()) {
    return false;
  }
  return out->ParseFromString(it->second);
}

// One traffic light message, kept in record-time order so TISE can ask for
// "the newest one recorded at or before t_F".
struct TrafficLightEntry {
  double record_time_sec = 0.0;
  int32_t sequence_num = -1;
  std::string content;
};

// Recover the frame construction time from stock published fields.
bool InferFrameTime(const ADCTrajectory& msg, double* frame_time) {
  if (msg.trajectory_point_size() == 0) {
    return false;
  }
  const double init_offset =
      msg.is_replan() ? kReplanInitOffsetSec : 0.0;
  *frame_time = msg.header().timestamp_sec() +
                msg.trajectory_point(0).relative_time() - init_offset;
  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <record_file> [output_dir]"
              << std::endl;
    return EXIT_FAILURE;
  }
  const std::string record_path = argv[1];
  const std::string out_dir =
      argc >= 3 ? std::string(argv[2]) : "/tmp/deft_record_replay_hybrid_tise";

  auto init_start = std::chrono::steady_clock::now();

  ::apollo::cyber::Init("deft_record_replay_hybrid_tise");

  std::string flag_file_path = "/apollo/modules/planning/conf/planning.conf";
  google::SetCommandLineOption("flagfile", flag_file_path.c_str());
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_test_base_map_filename = "base_map.bin";
  FLAGS_enable_reference_line_provider_thread = false;

  PlanningConfig config_;
  GetProtoFromFile("/apollo/modules/planning/conf/planning_config.pb.txt",
                   &config_);
  std::shared_ptr<DependencyInjector> injector_ =
      std::make_shared<DependencyInjector>();
  std::unique_ptr<PlanningBase> planning_(new OnLanePlanning(injector_));
  planning_->Init(config_);

  auto init_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> init_elapsed = init_end - init_start;

  apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
  apollo::cyber::Clock::SetNowInSeconds(0);

  // -- Pass 1: index only what debug.planning_data does NOT already embed
  // (prediction bodies, traffic lights), and collect every /apollo/planning
  // message that carries debug.planning_data. --
  SeqIndex prediction_index;
  std::vector<TrafficLightEntry> traffic_lights;
  std::vector<ADCTrajectory> frames;
  int planning_without_debug = 0;

  {
    RecordReader reader(record_path);
    if (!reader.IsValid()) {
      std::cerr << "Failed to open record file: " << record_path << std::endl;
      return EXIT_FAILURE;
    }
    RecordMessage message;
    while (reader.ReadMessage(&message)) {
      const std::string& channel = message.channel_name;
      if (channel == FLAGS_prediction_topic) {
        IndexBySequence<PredictionObstacles>(message.content,
                                             &prediction_index);
      } else if (channel == FLAGS_traffic_light_detection_topic) {
        TrafficLightDetection tl;
        if (tl.ParseFromString(message.content)) {
          TrafficLightEntry entry;
          entry.record_time_sec =
              static_cast<double>(message.time) / kNanosPerSecond;
          entry.sequence_num = tl.header().sequence_num();
          entry.content = message.content;
          traffic_lights.push_back(std::move(entry));
        }
      } else if (channel == FLAGS_planning_trajectory_topic) {
        ADCTrajectory msg;
        if (!msg.ParseFromString(message.content)) {
          continue;
        }
        if (msg.has_debug() && msg.debug().has_planning_data() &&
            msg.trajectory_point_size() > 0) {
          frames.push_back(msg);
        } else {
          ++planning_without_debug;
        }
      }
    }
  }

  // TISE needs record-time order. Sequence order and record-time order
  // coincided in every record checked, but sorting by time is what the
  // heuristic actually means.
  std::sort(traffic_lights.begin(), traffic_lights.end(),
            [](const TrafficLightEntry& a, const TrafficLightEntry& b) {
              return a.record_time_sec < b.record_time_sec;
            });

  std::cout << "indexed " << record_path
            << ": prediction=" << prediction_index.size()
            << " traffic_light=" << traffic_lights.size() << " -- found "
            << frames.size()
            << " /apollo/planning messages with debug.planning_data ("
            << planning_without_debug << " without)" << std::endl;

  if (frames.empty()) {
    std::cerr << "No /apollo/planning messages carrying debug.planning_data "
                 "were found. Planning must be recorded with "
                 "FLAGS_enable_record_debug enabled (it is on by default in "
                 "Apollo v7) for this metadata-free replay to work."
              << std::endl;
    return EXIT_FAILURE;
  }

  EnsureDirectory(out_dir);

  const bool realtime_pacing = std::getenv("DEFT_REPLAY_REALTIME") != nullptr;
  bool have_prev_timestamp = false;
  double prev_start_timestamp = 0.0;

  // TISE carry-state across frames: a sliding start index into the
  // time-ordered traffic light vector, plus the previously selected
  // sequence number to enforce monotonicity.
  size_t traffic_light_start_index = 0;
  bool have_prev_traffic_light = false;
  int32_t prev_traffic_light_seq = -1;

  std::chrono::duration<double> planning_duration(0);
  int replayed = 0;
  int skipped_missing_required_input = 0;
  int frames_without_traffic_light = 0;

  for (size_t i = 0; i < frames.size(); ++i) {
    const ADCTrajectory& original = frames[i];
    const auto& planning_data = original.debug().planning_data();

    double frame_time = 0.0;
    if (!InferFrameTime(original, &frame_time)) {
      std::cerr << "frame " << i
                << ": no trajectory point, cannot infer frame time; skipping."
                << std::endl;
      ++skipped_missing_required_input;
      continue;
    }

    // routing / chassis / localization come straight out of the planning
    // output -- no index, no lookup, no possibility of a missing message.
    if (!planning_data.has_routing() || !planning_data.has_chassis() ||
        !planning_data.has_adc_position() ||
        !planning_data.has_prediction_header()) {
      std::cerr << "frame " << i
                << ": debug.planning_data is missing one of routing/chassis/"
                   "adc_position/prediction_header; skipping."
                << std::endl;
      ++skipped_missing_required_input;
      continue;
    }

    PredictionObstacles prediction;
    if (!LookupBySequence(prediction_index,
                          planning_data.prediction_header().sequence_num(),
                          &prediction)) {
      std::cerr << "frame " << i << " (prediction_header="
                << planning_data.prediction_header().sequence_num()
                << "): prediction message not found in record; skipping."
                << std::endl;
      ++skipped_missing_required_input;
      continue;
    }

    // -- TISE for traffic light: newest message at or before t_F, with a
    // non-decreasing sequence number across frames. --
    bool found_traffic_light = false;
    int32_t traffic_light_seq = -1;
    size_t best_index = traffic_light_start_index;
    for (size_t j = traffic_light_start_index; j < traffic_lights.size();
         ++j) {
      if (traffic_lights[j].record_time_sec <= frame_time) {
        traffic_light_seq = traffic_lights[j].sequence_num;
        found_traffic_light = true;
        best_index = j;
      } else {
        break;  // sorted by record time
      }
    }

    if (have_prev_traffic_light) {
      if (!found_traffic_light) {
        traffic_light_seq = prev_traffic_light_seq;
        found_traffic_light = true;
      } else {
        traffic_light_seq = std::max(traffic_light_seq, prev_traffic_light_seq);
      }
    }

    if (found_traffic_light) {
      have_prev_traffic_light = true;
      prev_traffic_light_seq = traffic_light_seq;
      traffic_light_start_index = best_index;
    }

    TrafficLightDetection traffic_light;
    bool has_traffic_light = false;
    if (found_traffic_light) {
      for (const auto& entry : traffic_lights) {
        if (entry.sequence_num == traffic_light_seq) {
          has_traffic_light = traffic_light.ParseFromString(entry.content);
          break;
        }
      }
    }
    if (!has_traffic_light) {
      ++frames_without_traffic_light;
    }

    if (realtime_pacing) {
      if (have_prev_timestamp) {
        const double gap = frame_time - prev_start_timestamp;
        if (gap > 0.0 && gap < 5.0) {
          std::this_thread::sleep_for(std::chrono::duration<double>(gap));
        }
      }
      prev_start_timestamp = frame_time;
      have_prev_timestamp = true;
    }

    apollo::cyber::Clock::SetNowInSeconds(frame_time);

    LocalView local_view_;
    local_view_.routing =
        std::make_shared<RoutingResponse>(planning_data.routing());
    local_view_.chassis = std::make_shared<Chassis>(planning_data.chassis());
    local_view_.localization_estimate =
        std::make_shared<LocalizationEstimate>(planning_data.adc_position());
    local_view_.prediction_obstacles =
        std::make_shared<PredictionObstacles>(prediction);
    if (has_traffic_light) {
      local_view_.traffic_light =
          std::make_shared<TrafficLightDetection>(traffic_light);
    }
    // stories and pad are not recoverable without the `deft` block.

    ADCTrajectory replayed_trajectory;
    const auto plan_start = std::chrono::steady_clock::now();
    planning_->RunOnce(local_view_, &replayed_trajectory);
    const auto plan_end = std::chrono::steady_clock::now();
    planning_duration += (plan_end - plan_start);

    const std::string frame_dir = out_dir + "/" + std::to_string(i);
    EnsureDirectory(frame_dir);
    SetProtoToBinaryFile(replayed_trajectory, frame_dir + "/deft.bin");
    SetProtoToBinaryFile(original, frame_dir + "/original.bin");

    ++replayed;
    std::cout << "frame " << i << "/" << frames.size() << " replayed"
              << (has_traffic_light ? "" : " [no traffic_light]")
              << std::endl;
  }

  auto final_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> total_elapsed = final_end - init_start;

  std::cout << "replayed " << replayed << "/" << frames.size() << " frames ("
            << skipped_missing_required_input
            << " skipped for missing required input, "
            << frames_without_traffic_light << " without traffic_light)"
            << std::endl;
  std::cout << "output written under " << out_dir << std::endl;
  std::cout << "INIT TIME: " << init_elapsed.count() << " seconds" << std::endl;
  std::cout << "TOTAL TIME: " << total_elapsed.count() << " seconds"
            << std::endl;
  std::cout << "PLANNING TIME: " << planning_duration.count() << " seconds"
            << std::endl;
  return 0;
}
