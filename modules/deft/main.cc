#include <iostream>
#include <map>
#include <string>

#include "cyber/proto/record.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/storytelling/proto/story.pb.h"

#include "cyber/cyber.h"
#include "cyber/init.h"
#include "cyber/message/raw_message.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"
#include "cyber/time/clock.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/planning_base.h"

using ::apollo::cyber::common::GetProtoFromFile;
using ::apollo::cyber::message::RawMessage;
using ::apollo::cyber::record::RecordMessage;
using ::apollo::cyber::record::RecordReader;
using ::apollo::cyber::record::RecordWriter;

using ::apollo::canbus::Chassis;
using ::apollo::common::TrajectoryPoint;
using ::apollo::localization::LocalizationEstimate;
using ::apollo::perception::TrafficLightDetection;
using ::apollo::prediction::PredictionObstacles;
using ::apollo::routing::RoutingResponse;
using ::apollo::storytelling::Stories;

using ::apollo::planning::ADCTrajectory;
using ::apollo::planning::DependencyInjector;
using ::apollo::planning::LocalView;
using ::apollo::planning::OnLanePlanning;
using ::apollo::planning::PlanningBase;
using ::apollo::planning::PlanningConfig;
using ::apollo::planning_internal::PlanningData;

int main(int argc, char *argv[]) {
  ::apollo::cyber::Init("deft_icse_24");

  // Initialize Planning
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
  std::unique_ptr<PlanningBase> planning_ =
      std::unique_ptr<PlanningBase>(new OnLanePlanning(injector_));
  planning_->Init(config_);

  // DeFT

  apollo::cyber::Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
  apollo::cyber::Clock::SetNowInSeconds(0);

  std::string deft_tmp_dir = "/apollo/modules/deft/testdata";

  int input_seq_num = 0;

  while (true) {
    // check if 0_planning.pb.txt exists
    std::string input_file_name =
        deft_tmp_dir + "/" + std::to_string(input_seq_num) + "/planning.pb.txt";
    std::ifstream input_file(input_file_name);
    if (!input_file.good()) {
      break;
    }
    input_file.close();

    // load inputs to planning module
    RoutingResponse routing;
    Chassis chassis;
    LocalizationEstimate adc_position;
    PredictionObstacles prediction;
    TrafficLightDetection tld;
    Stories stories;
    ADCTrajectory planning;

    apollo::cyber::common::GetProtoFromFile(
        deft_tmp_dir + "/" + std::to_string(input_seq_num) + "/routing.pb.txt",
        &routing);
    apollo::cyber::common::GetProtoFromFile(
        deft_tmp_dir + "/" + std::to_string(input_seq_num) + "/chassis.pb.txt",
        &chassis);
    apollo::cyber::common::GetProtoFromFile(deft_tmp_dir + "/" +
                                                std::to_string(input_seq_num) +
                                                "/localization.pb.txt",
                                            &adc_position);
    apollo::cyber::common::GetProtoFromFile(deft_tmp_dir + "/" +
                                                std::to_string(input_seq_num) +
                                                "/prediction.pb.txt",
                                            &prediction);
    apollo::cyber::common::GetProtoFromFile(deft_tmp_dir + "/" +
                                                std::to_string(input_seq_num) +
                                                "/traffic_light.pb.txt",
                                            &tld);
    // apollo::cyber::common::GetProtoFromFile(deft_tmp_dir + "/" +
    //                                             std::to_string(input_seq_num) +
    //                                             "/stories.pb.txt",
    //                                         &stories);
    apollo::cyber::common::GetProtoFromFile(
        deft_tmp_dir + "/" + std::to_string(input_seq_num) + "/planning.pb.txt",
        &planning);

    apollo::cyber::Clock::SetNowInSeconds(planning.header().timestamp_sec());

    LocalView local_view_;
    local_view_.routing = std::make_shared<RoutingResponse>(routing);
    local_view_.chassis = std::make_shared<Chassis>(chassis);
    local_view_.localization_estimate =
        std::make_shared<LocalizationEstimate>(adc_position);
    local_view_.prediction_obstacles =
        std::make_shared<PredictionObstacles>(prediction);
    local_view_.traffic_light = std::make_shared<TrafficLightDetection>(tld);
    local_view_.stories = std::make_shared<Stories>(stories);
    
    ADCTrajectory adc_trajectory_pb;
    planning_->RunOnce(local_view_, &adc_trajectory_pb);

    std::string output_file_name =
        deft_tmp_dir + "/" + std::to_string(input_seq_num) + "/deft.pb.txt";

    apollo::cyber::common::SetProtoToASCIIFile(adc_trajectory_pb, output_file_name);
    // std::ofstream output(output_file_name);
    // output << json;
    // output.close();

    input_seq_num++;
  }

  std::cout << "stopped at input_seq_num: " << input_seq_num << std::endl;
  return 0;
}

// bazel run //modules/coverage:main /apollo/modules/coverage/test.00000
// bazel run //modules/coverage:main_test /apollo/modules/coverage/test.00000

// source scripts/apollo_base.sh
// bazel coverage -s --combined_report=lcov
// --instrumentation_filter="^//modules/planning[/:]"
// //modules/coverage:main_test bazel coverage -s --combined_report=lcov
// --instrumentation_filter="^//modules/planning[/:]"
// --test_arg="/apollo/modules/coverage/testData/0.00000"
// //modules/coverage:main_test genhtml --output genhtml0 "$(bazel info
// output_path)/_coverage/_coverage_report.dat"