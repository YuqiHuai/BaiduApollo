## Auto generated by `proto_build_generator.py`
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "frame_proto",
    srcs = ["frame.proto"],
)

proto_library(
    name = "static_info_proto",
    srcs = ["static_info.proto"],
    deps = [
        "//modules/canbus/proto:canbus_conf_proto",
        "//modules/common_msgs/chassis_msgs:chassis_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_proto",
        # "//modules/control/proto:control_conf_proto",
        "//modules/common_msgs/routing_msgs:routing_proto",
    ],
)

apollo_package()

