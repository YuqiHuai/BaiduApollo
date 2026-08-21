#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
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
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/deft/deft_planning.h"
#include "modules/planning/common/dependency_injector.h"

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

using ::apollo::deft::DeFTPlanning;
using ::apollo::deft::PlannerCheckpoint;
using ::apollo::planning::ADCTrajectory;
using ::apollo::planning::DependencyInjector;
using ::apollo::planning::LocalView;
using ::apollo::planning::PlanningConfig;

namespace {

constexpr double kReplanInitOffsetSec = 0.1;
constexpr double kNanosPerSecond = 1e9;
constexpr double kEligibilityEpsilonSec = 1e-3;
constexpr int kNumDataPoints = 10;
constexpr int kDefaultMaxRank = 25;
constexpr double kDefaultThreshold = 1e-4;

struct Entry {
  double time = 0.0;
  int32_t seq = -1;
  std::string content;
};

using Stream = std::vector<Entry>;

template <typename T>
void Index(const std::string& content, double t, Stream* stream) {
  T msg;
  if (!msg.ParseFromString(content)) return;
  stream->push_back(Entry{t, msg.header().sequence_num(), content});
}

const Entry* Newest(const Stream& stream, double tf, double not_before) {
  const Entry* best = nullptr;
  for (const auto& e : stream) {
    if (e.time > tf + kEligibilityEpsilonSec) break;
    if (e.time < not_before) continue;
    best = &e;
  }
  return best;
}

std::vector<const Entry*> Candidates(const Stream& stream, double tf,
                                     double not_before, int max_rank) {
  std::vector<const Entry*> out;
  for (auto it = stream.rbegin(); it != stream.rend(); ++it) {
    if (it->time > tf + kEligibilityEpsilonSec) continue;
    if (it->time < not_before) break;
    out.push_back(&(*it));
    if (static_cast<int>(out.size()) >= max_rank) break;
  }
  return out;
}

double Interp(const std::vector<double>& xs, const std::vector<double>& ys,
              double x) {
  if (xs.empty()) return 0.0;
  if (x <= xs.front()) return ys.front();
  if (x >= xs.back()) return ys.back();
  const size_t i = std::lower_bound(xs.begin(), xs.end(), x) - xs.begin();
  const double x0 = xs[i - 1], x1 = xs[i], y0 = ys[i - 1], y1 = ys[i];
  return (x1 == x0) ? y0 : y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

double ReproduceError(const ADCTrajectory& a, const ADCTrajectory& b) {
  if (a.trajectory_point_size() == 0 || b.trajectory_point_size() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  std::vector<double> at, ax, ay, bt, bx, by;
  for (const auto& p : a.trajectory_point()) {
    at.push_back(p.relative_time());
    ax.push_back(p.path_point().x());
    ay.push_back(p.path_point().y());
  }
  for (const auto& p : b.trajectory_point()) {
    bt.push_back(p.relative_time());
    bx.push_back(p.path_point().x());
    by.push_back(p.path_point().y());
  }
  const double a0 = at.front(), b0 = bt.front();
  const int min_duration =
      static_cast<int>(std::min(at.back() - at.front(), bt.back() - bt.front()));
  double total = 0.0;
  for (int i = 0; i < kNumDataPoints; ++i) {
    const double t = min_duration * static_cast<double>(i) / (kNumDataPoints - 1);
    const double dx = Interp(at, ax, t + a0) - Interp(bt, bx, t + b0);
    const double dy = Interp(at, ay, t + a0) - Interp(bt, by, t + b0);
    total += dx * dx + dy * dy;
  }
  return std::sqrt(total);
}

bool InferFrameTime(const ADCTrajectory& msg, double* tf) {
  if (msg.trajectory_point_size() == 0) return false;
  *tf = msg.header().timestamp_sec() + msg.trajectory_point(0).relative_time() -
        (msg.is_replan() ? kReplanInitOffsetSec : 0.0);
  return true;
}

double EnvDouble(const char* name, double fallback) {
  const char* v = std::getenv(name);
  return v ? std::atof(v) : fallback;
}

int EnvInt(const char* name, int fallback) {
  const char* v = std::getenv(name);
  return v ? std::atoi(v) : fallback;
}

}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <record_file> [output_dir]"
              << std::endl;
    return EXIT_FAILURE;
  }
  const std::string record_path = argv[1];
  const std::string out_dir =
      argc >= 3 ? std::string(argv[2]) : "/tmp/deft_search";
  const double threshold = EnvDouble("DEFT_THRESHOLD", kDefaultThreshold);
  const int max_rank = EnvInt("DEFT_MAX_RANK", kDefaultMaxRank);

  ::apollo::cyber::Init("deft_search");
  google::SetCommandLineOption(
      "flagfile", "/apollo/modules/planning/conf/planning.conf");
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_test_base_map_filename = "base_map.bin";
  FLAGS_enable_reference_line_provider_thread = false;

  PlanningConfig config;
  GetProtoFromFile("/apollo/modules/planning/conf/planning_config.pb.txt",
                   &config);
  auto injector = std::make_shared<DependencyInjector>();
  std::unique_ptr<DeFTPlanning> planning(new DeFTPlanning(injector));
  planning->Init(config);

  apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
  apollo::cyber::Clock::SetNowInSeconds(0);

  Stream routing_s, chassis_s, localization_s, prediction_s, traffic_s;
  std::vector<ADCTrajectory> frames;
  bool started = false;
  {
    RecordReader reader(record_path);
    if (!reader.IsValid()) {
      std::cerr << "Failed to open record: " << record_path << std::endl;
      return EXIT_FAILURE;
    }
    RecordMessage m;
    while (reader.ReadMessage(&m)) {
      const double t = static_cast<double>(m.time) / kNanosPerSecond;
      if (m.channel_name == FLAGS_routing_response_topic) {
        Index<RoutingResponse>(m.content, t, &routing_s);
      } else if (m.channel_name == FLAGS_chassis_topic) {
        Index<Chassis>(m.content, t, &chassis_s);
      } else if (m.channel_name == FLAGS_localization_topic) {
        Index<LocalizationEstimate>(m.content, t, &localization_s);
      } else if (m.channel_name == FLAGS_prediction_topic) {
        Index<PredictionObstacles>(m.content, t, &prediction_s);
      } else if (m.channel_name == FLAGS_traffic_light_detection_topic) {
        Index<TrafficLightDetection>(m.content, t, &traffic_s);
      } else if (m.channel_name == FLAGS_planning_trajectory_topic) {
        ADCTrajectory a;
        if (!a.ParseFromString(m.content) || a.trajectory_point_size() == 0) {
          continue;
        }

        if (!started && !a.decision().main_decision().has_not_ready()) {
          started = true;
        }
        if (started) frames.push_back(a);
      }
    }
  }
  auto by_time = [](const Entry& a, const Entry& b) { return a.time < b.time; };
  std::sort(routing_s.begin(), routing_s.end(), by_time);
  std::sort(chassis_s.begin(), chassis_s.end(), by_time);
  std::sort(localization_s.begin(), localization_s.end(), by_time);
  std::sort(prediction_s.begin(), prediction_s.end(), by_time);
  std::sort(traffic_s.begin(), traffic_s.end(), by_time);

  std::cerr << "deft_search: " << frames.size() << " cycles; chassis="
            << chassis_s.size() << " localization=" << localization_s.size()
            << " prediction=" << prediction_s.size() << " traffic_light="
            << traffic_s.size() << " routing=" << routing_s.size()
            << "; threshold=" << threshold << " max_rank=" << max_rank
            << std::endl;
  EnsureDirectory(out_dir);

  double last_vehicle_time = 0.0;
  int32_t prev_traffic_seq = -1;
  double prev_traffic_time = 0.0;

  long total_trials = 0;
  int solved = 0, failed = 0, widened = 0;
  std::vector<int> failed_frames;

  printf("frame,planning_seq,tf,trials,accepted_rank,paired,error\n");

  for (size_t i = 0; i < frames.size(); ++i) {
    const ADCTrajectory& recorded = frames[i];
    double tf = 0.0;
    if (!InferFrameTime(recorded, &tf)) continue;

    const Entry* routing_e = Newest(routing_s, tf, 0.0);
    const Entry* prediction_e = Newest(prediction_s, tf, 0.0);
    if (!routing_e || !prediction_e) {
      std::cerr << "frame " << i << ": no routing/prediction candidate"
                << std::endl;
      ++failed;
      failed_frames.push_back(static_cast<int>(i));
      continue;
    }
    RoutingResponse routing;
    PredictionObstacles prediction;
    routing.ParseFromString(routing_e->content);
    prediction.ParseFromString(prediction_e->content);

    const Entry* traffic_e = Newest(traffic_s, tf, prev_traffic_time);
    TrafficLightDetection traffic;
    bool has_traffic = false;
    if (traffic_e && traffic_e->seq >= prev_traffic_seq) {
      has_traffic = traffic.ParseFromString(traffic_e->content);
    } else if (prev_traffic_seq >= 0) {

      for (const auto& e : traffic_s) {
        if (e.seq == prev_traffic_seq) {
          has_traffic = traffic.ParseFromString(e.content);
          traffic_e = &e;
          break;
        }
      }
    }

    auto build = [&](const Chassis& ch, const LocalizationEstimate& lo) {
      LocalView lv;
      lv.routing = std::make_shared<RoutingResponse>(routing);
      lv.chassis = std::make_shared<Chassis>(ch);
      lv.localization_estimate = std::make_shared<LocalizationEstimate>(lo);
      lv.prediction_obstacles =
          std::make_shared<PredictionObstacles>(prediction);
      if (has_traffic) {
        lv.traffic_light = std::make_shared<TrafficLightDetection>(traffic);
      }
      return lv;
    };

    auto cc = Candidates(chassis_s, tf, last_vehicle_time, max_rank);
    auto lc = Candidates(localization_s, tf, last_vehicle_time, max_rank);

    PlannerCheckpoint cp;
    planning->Checkpoint(&cp);

    int trials = 0, accepted_rank = -1;
    ADCTrajectory accepted_out;
    LocalView accepted_view;
    bool accepted_paired = true;
    double accepted_error = std::numeric_limits<double>::infinity();
    const Entry *best_ch = nullptr, *best_lo = nullptr;
    double best_error = std::numeric_limits<double>::infinity();
    const Entry *fallback_ch = nullptr, *fallback_lo = nullptr;

    auto try_pair = [&](const Entry* ch_e, const Entry* lo_e, int rank,
                        bool paired) -> bool {
      Chassis ch;
      LocalizationEstimate lo;
      if (!ch.ParseFromString(ch_e->content) ||
          !lo.ParseFromString(lo_e->content)) {
        return false;
      }
      ADCTrajectory out;
      const LocalView lv = build(ch, lo);
      planning->Trial(lv, tf, &out);
      ++trials;
      const double err = ReproduceError(out, recorded);
      if (err < best_error) {
        best_error = err;
        fallback_ch = ch_e;
        fallback_lo = lo_e;
      }
      if (err <= threshold) {

        accepted_rank = rank;
        accepted_paired = paired;
        accepted_error = err;
        best_ch = ch_e;
        best_lo = lo_e;
        accepted_out = out;
        accepted_view = lv;
        return true;
      }
      planning->Rollback(cp);
      return false;
    };

    const size_t n = std::min(cc.size(), lc.size());
    for (size_t k = 0; k < n && accepted_rank < 0; ++k) {
      try_pair(cc[k], lc[k], static_cast<int>(k) + 1, true);
    }

    if (accepted_rank < 0) {
      ++widened;
      for (size_t k = 0; k < n && accepted_rank < 0; ++k) {
        if (k + 1 < lc.size() && try_pair(cc[k], lc[k + 1],
                                          static_cast<int>(k) + 1, false)) {
          break;
        }
        if (k + 1 < cc.size() && try_pair(cc[k + 1], lc[k],
                                          static_cast<int>(k) + 1, false)) {
          break;
        }
      }
    }

    total_trials += trials;

    const Entry* commit_ch = (accepted_rank > 0) ? best_ch : fallback_ch;
    const Entry* commit_lo = (accepted_rank > 0) ? best_lo : fallback_lo;
    if (!commit_ch || !commit_lo) {
      std::cerr << "frame " << i << ": no usable vehicle-state candidate"
                << std::endl;
      ++failed;
      failed_frames.push_back(static_cast<int>(i));
      continue;
    }
    if (accepted_rank < 0) {
      ++failed;
      failed_frames.push_back(static_cast<int>(i));
      std::cerr << "frame " << i << ": SEARCH FAILED after " << trials
                << " trials; best error " << best_error
                << " (committing best candidate so the replay can continue)"
                << std::endl;

    } else {
      ++solved;
    }

    ADCTrajectory out;
    if (accepted_rank > 0) {

      out = accepted_out;
      planning->NoteCommitted(accepted_view);
    } else {

      Chassis ch;
      LocalizationEstimate lo;
      ch.ParseFromString(commit_ch->content);
      lo.ParseFromString(commit_lo->content);
      planning->Commit(build(ch, lo), tf, &out);
    }

    last_vehicle_time = std::min(commit_ch->time, commit_lo->time);
    if (has_traffic && traffic_e) {
      prev_traffic_seq = traffic_e->seq;
      prev_traffic_time = traffic_e->time;
    }

    const std::string frame_dir = out_dir + "/" + std::to_string(i);
    EnsureDirectory(frame_dir);
    SetProtoToBinaryFile(out, frame_dir + "/deft.bin");
    SetProtoToBinaryFile(recorded, frame_dir + "/original.bin");

    printf("%zu,%d,%.9f,%d,%d,%d,%.9e\n", i,
           recorded.header().sequence_num(), tf, trials, accepted_rank,
           accepted_paired ? 1 : 0,
           (accepted_rank > 0) ? accepted_error : best_error);
    fflush(stdout);
  }

  std::cerr << "\n=== deft_search summary ===" << std::endl;
  std::cerr << "cycles          : " << frames.size() << std::endl;
  std::cerr << "solved          : " << solved << std::endl;
  std::cerr << "failed          : " << failed << std::endl;
  std::cerr << "widened         : " << widened << std::endl;
  std::cerr << "planner trials  : " << total_trials << "  ("
            << (frames.empty() ? 0.0
                               : static_cast<double>(total_trials) /
                                     static_cast<double>(frames.size()))
            << " per cycle)" << std::endl;
  if (!failed_frames.empty()) {
    std::cerr << "failed frames   :";
    for (int f : failed_frames) std::cerr << " " << f;
    std::cerr << std::endl;
  }
  std::cerr << "output          : " << out_dir << std::endl;
  return failed == 0 ? 0 : 2;
}
