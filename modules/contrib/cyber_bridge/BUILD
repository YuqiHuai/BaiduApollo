load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package", "apollo_cc_binary")
load("//tools/proto:proto.bzl", "apollo_py_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "cyber_bridge",
    srcs = ["bridge.cc"],
    deps = [
        ":contrib_cyber_bridge",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_library(
    name = "contrib_cyber_bridge",
    srcs = [
        "client.cc",
        "clients.cc",
        "node.cc",
        "server.cc",
    ],
    hdrs = [
        "client.h",
        "clients.h",
        "node.h",
        "server.h",
    ],
    deps = [
        "//cyber",
        "@boost",
    ],
)

apollo_py_binary(
    name = "service_bridge",
    srcs = ["service_bridge.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/external_command_msgs:action_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:command_status_py_pb2",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_py_pb2",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_py_pb2",
    ],
)

apollo_package()

cpplint()
