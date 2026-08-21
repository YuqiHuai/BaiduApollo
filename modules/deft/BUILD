load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools/platform:build_defs.bzl", "copts_if_teleop")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "//modules/planning:planning_component_lib",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/storytelling/proto:story_cc_proto"

    ],
    data = [
        "//modules/map/data:borregas_ave",
    ]
)

cc_binary(
    name = "record_main",
    srcs = ["record_main.cc"],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/planning:planning_component_lib",
        "//modules/planning/proto:pad_msg_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/storytelling/proto:story_cc_proto",
    ],
    data = [
        "//modules/map/data:borregas_ave",
    ],
)

cc_binary(
    name = "record_main_hybrid_tise",
    srcs = ["record_main_hybrid_tise.cc"],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/planning:planning_component_lib",
        "//modules/planning/proto:planning_internal_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
    ],
    data = [
        "//modules/map/data:borregas_ave",
    ],
)

cc_binary(
    name = "deft_search",
    srcs = ["deft_search.cc", "deft_planning.h"],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/planning:planning_component_lib",
        "//modules/planning/proto:planning_status_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
    ],
    data = [
        "//modules/map/data:borregas_ave",
    ],
)

cc_test(
    name = "main_test",
    srcs = ["main.cc"],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "//modules/planning:planning_component_lib",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/routing/proto:routing_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/storytelling/proto:story_cc_proto"
    ],
    data = [
        "//modules/map/data:borregas_ave",
    ]
)