// DeFT, record-file variant: replays a recorded Planning trial frame by
// frame directly from a cyber_record file, instead of from pre-extracted
// per-frame testdata/<n>/*.bin files (see main.cc).
//
// Each /apollo/planning message produced by an instrumented live run
// carries a `deft` block (see modules/planning/proto/planning.proto) with
// the header sequence_num of every input that fed that specific planning
// cycle: routing, chassis, localization, prediction, and (when present)
// traffic_light/stories/pad. This tool makes one pass over the record to
// index every relevant topic by header.sequence_num(), then a second pass
// that, for every /apollo/planning message with a `deft` block, looks up
// the exact inputs that cycle used, sets the mock clock to that cycle's
// recorded start_timestamp, and calls the same
// PlanningBase::RunOnce()/OnLanePlanning entry point production code and
// modules/deft/main.cc both use -- so the replay exercises Planning
// exactly as it ran live, without needing a separate offline extraction
// step to produce testdata/.

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
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

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
using ::apollo::planning::PadMessage;
using ::apollo::prediction::PredictionObstacles;
using ::apollo::routing::RoutingResponse;
using ::apollo::storytelling::Stories;

using ::apollo::planning::ADCTrajectory;
using ::apollo::planning::DependencyInjector;
using ::apollo::planning::LocalView;
using ::apollo::planning::OnLanePlanning;
using ::apollo::planning::PlanningBase;
using ::apollo::planning::PlanningConfig;

namespace {

// header.sequence_num() -> serialized proto bytes, for one topic.
using SeqIndex = std::unordered_map<int32_t, std::string>;

template <typename T>
void IndexBySequence(const std::string& content, SeqIndex* index) {
  T msg;
  if (msg.ParseFromString(content)) {
    (*index)[msg.header().sequence_num()] = content;
  }
}

// Looks up `seq` in `index` and parses it into `out`. A negative `seq`
// (the DeFT proto's default for an absent optional input, e.g. no pad
// message ever received yet) is not an error -- it just means this input
// was legitimately absent for this cycle, matching the live run.
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

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <record_file> [output_dir]"
              << std::endl;
    return EXIT_FAILURE;
  }
  const std::string record_path = argv[1];
  const std::string out_dir =
      argc >= 3 ? std::string(argv[2]) : "/tmp/deft_record_replay";

  auto init_start = std::chrono::steady_clock::now();

  ::apollo::cyber::Init("deft_record_replay");

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

  // -- Pass 1: index every input topic by header.sequence_num(), and
  // collect (in original order) every /apollo/planning message that
  // carries a `deft` block. --
  SeqIndex routing_index, chassis_index, localization_index,
      prediction_index, traffic_light_index, stories_index, pad_index;
  std::vector<ADCTrajectory> deft_frames;

  {
    RecordReader reader(record_path);
    if (!reader.IsValid()) {
      std::cerr << "Failed to open record file: " << record_path
                << std::endl;
      return EXIT_FAILURE;
    }
    RecordMessage message;
    while (reader.ReadMessage(&message)) {
      const std::string& channel = message.channel_name;
      if (channel == FLAGS_routing_response_topic) {
        IndexBySequence<RoutingResponse>(message.content, &routing_index);
      } else if (channel == FLAGS_chassis_topic) {
        IndexBySequence<Chassis>(message.content, &chassis_index);
      } else if (channel == FLAGS_localization_topic) {
        IndexBySequence<LocalizationEstimate>(message.content,
                                              &localization_index);
      } else if (channel == FLAGS_prediction_topic) {
        IndexBySequence<PredictionObstacles>(message.content,
                                             &prediction_index);
      } else if (channel == FLAGS_traffic_light_detection_topic) {
        IndexBySequence<TrafficLightDetection>(message.content,
                                               &traffic_light_index);
      } else if (channel == FLAGS_storytelling_topic) {
        IndexBySequence<Stories>(message.content, &stories_index);
      } else if (channel == FLAGS_planning_pad_topic) {
        IndexBySequence<PadMessage>(message.content, &pad_index);
      } else if (channel == FLAGS_planning_trajectory_topic) {
        ADCTrajectory msg;
        if (msg.ParseFromString(message.content) && msg.has_deft()) {
          deft_frames.push_back(msg);
        }
      }
    }
  }

  std::cout << "indexed " << record_path << ": routing="
            << routing_index.size() << " chassis=" << chassis_index.size()
            << " localization=" << localization_index.size()
            << " prediction=" << prediction_index.size()
            << " traffic_light=" << traffic_light_index.size()
            << " stories=" << stories_index.size()
            << " pad=" << pad_index.size() << " -- found "
            << deft_frames.size()
            << " /apollo/planning messages with an embedded deft block"
            << std::endl;

  if (deft_frames.empty()) {
    std::cerr << "No /apollo/planning messages with a `deft` block were "
                 "found. This record may predate the DeFT instrumentation "
                 "(planning.proto's DeFT message / on_lane_planning.cc's "
                 "logging of it) -- re-record with a build that includes "
                 "it."
              << std::endl;
    return EXIT_FAILURE;
  }

  EnsureDirectory(out_dir);

  // -- Pass 2: replay each frame, in original order, on one continuous
  // PlanningBase instance -- matching main.cc's single-instance replay
  // loop, so the stitching-state behavior (see
  // bugs/apollo-v7-simcontrol-trajectory-stitcher-flicker/) is identical
  // to what main.cc does with testdata/. --
  // Optional real-time pacing: sleep between frames to match the real
  // wall-clock gap the original recording had, instead of replaying as
  // fast as possible. Tests whether any hidden real-time-sensitive logic
  // (a budget, a background-thread race, anything not driven by the
  // mocked Clock) explains divergence that appears even when every
  // Clock-visible input and the mocked timestamp are reproduced exactly.
  const bool realtime_pacing = std::getenv("DEFT_REPLAY_REALTIME") != nullptr;
  bool have_prev_timestamp = false;
  double prev_start_timestamp = 0.0;

  std::chrono::duration<double> planning_duration(0);
  int replayed = 0;
  int skipped_missing_required_input = 0;

  for (size_t i = 0; i < deft_frames.size(); ++i) {
    const ADCTrajectory& original = deft_frames[i];
    const auto& deft_meta = original.deft();

    RoutingResponse routing;
    Chassis chassis;
    LocalizationEstimate localization;
    PredictionObstacles prediction;

    const bool have_required =
        LookupBySequence(routing_index, deft_meta.routing_header(),
                         &routing) &&
        LookupBySequence(chassis_index, deft_meta.chassis_header(),
                         &chassis) &&
        LookupBySequence(localization_index, deft_meta.localization_header(),
                         &localization) &&
        LookupBySequence(prediction_index, deft_meta.prediction_header(),
                         &prediction);
    if (!have_required) {
      std::cerr << "frame " << i << " (routing_header="
                << deft_meta.routing_header()
                << ", chassis_header=" << deft_meta.chassis_header()
                << ", localization_header="
                << deft_meta.localization_header()
                << ", prediction_header=" << deft_meta.prediction_header()
                << "): could not find one of the required inputs in the "
                   "record; skipping."
              << std::endl;
      ++skipped_missing_required_input;
      continue;
    }

    TrafficLightDetection traffic_light;
    const bool has_traffic_light =
        deft_meta.has_traffic_light_header() &&
        LookupBySequence(traffic_light_index,
                         deft_meta.traffic_light_header(), &traffic_light);

    Stories stories;
    const bool has_stories =
        deft_meta.has_stories_header() &&
        LookupBySequence(stories_index, deft_meta.stories_header(),
                         &stories);

    PadMessage pad;
    const bool has_pad =
        deft_meta.has_pad_header() &&
        LookupBySequence(pad_index, deft_meta.pad_header(), &pad);

    if (realtime_pacing) {
      if (have_prev_timestamp) {
        const double gap = deft_meta.start_timestamp() - prev_start_timestamp;
        if (gap > 0.0 && gap < 5.0) {  // sanity bound against bogus gaps
          std::this_thread::sleep_for(std::chrono::duration<double>(gap));
        }
      }
      prev_start_timestamp = deft_meta.start_timestamp();
      have_prev_timestamp = true;
    }

    apollo::cyber::Clock::SetNowInSeconds(deft_meta.start_timestamp());

    LocalView local_view_;
    local_view_.routing = std::make_shared<RoutingResponse>(routing);
    local_view_.chassis = std::make_shared<Chassis>(chassis);
    local_view_.localization_estimate =
        std::make_shared<LocalizationEstimate>(localization);
    local_view_.prediction_obstacles =
        std::make_shared<PredictionObstacles>(prediction);
    if (has_traffic_light) {
      local_view_.traffic_light =
          std::make_shared<TrafficLightDetection>(traffic_light);
    }
    if (has_stories) {
      local_view_.stories = std::make_shared<Stories>(stories);
    }
    if (has_pad) {
      local_view_.pad_msg = std::make_shared<PadMessage>(pad);
    }

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
    std::cout << "frame " << i << "/" << deft_frames.size() << " replayed"
              << (has_traffic_light ? "" : " [no traffic_light]")
              << (has_stories ? "" : " [no stories]")
              << (has_pad ? " [pad]" : "") << std::endl;
  }

  auto final_end = std::chrono::steady_clock::now();
  std::chrono::duration<double> total_elapsed = final_end - init_start;

  std::cout << "replayed " << replayed << "/" << deft_frames.size()
            << " frames (" << skipped_missing_required_input
            << " skipped for missing required input)" << std::endl;
  std::cout << "output written under " << out_dir << std::endl;
  std::cout << "INIT TIME: " << init_elapsed.count() << " seconds"
            << std::endl;
  std::cout << "TOTAL TIME: " << total_elapsed.count() << " seconds"
            << std::endl;
  std::cout << "PLANNING TIME: " << planning_duration.count() << " seconds"
            << std::endl;
  return 0;
}
