# cyber-bridge

## Introduction

This is bridge that exposes custom TCP socket for accepting and transmitting Cyber messages.

## Directory Structure
```shell
modules/contrib/cyber_bridge/
├── bridge.cc
├── BUILD
├── client.cc
├── client.h
├── clients.cc
├── clients.h
├── cyber-bridge.BUILD
├── cyberfile.xml
├── LICENSE
├── node.cc
├── node.h
├── README.md
├── server.cc
└── server.h
```

## Building

Run the build inside docker.

```shell
cd /apollo && bash apollo.sh build contrib                      # in source env
cd /apollo_workspace && buildtool build -p modules/contrib      # in package management env
```

## Running

```shell
cd /apollo && ./bazel-bin/module/contrib/cyber_bridge/cyber_bridge  # in source env
cyber_bridge                                                        # in package management env
```

For extra logging:

```shell
GLOG_v=4 GLOG_logtostderr=1 ./bazel-bin/modules/contrib/cyber_bridge/cyber_bridge # in source env
GLOG_v=4 GLOG_logtostderr=1 cyber_bridge                                          # in package management env 
```

Add extra `-port 9090` argument for custom port (9090 is default).

## Example

In one terminal launch `cyber_bridge`:

```shell
cd /apollo && ./bazel-bin/module/contrib/cyber_bridge/cyber_bridge  # in source env
cyber_bridge                                                        # in package management env
```

In another terminal launch example talker:

```shell
cd /apollo && ./bazel-bin/cyber/python/cyber_py3/examples/talker    # in source env
talker                                                              # in package management env     
```

In one more terminal launch example listener:

```shell
cd /apollo && ./bazel-bin/cyber/python/cyber_py3/examples/listener  # in source env
listener                                                            # in package management env 
```

Now you should observe talker and listener sending & receiving message with incrementing integer.

## External Command Service Bridge

`service_bridge.py` exposes selected external-command Cyber services as HTTP
JSON endpoints. It binds to localhost by default because it does not provide
authentication.

Run it after starting the external-command process:

```shell
./bazel-bin/modules/contrib/cyber_bridge/service_bridge
```

Send a lane-follow command:

```shell
curl -X POST http://127.0.0.1:9091/lane_follow \
  -H 'Content-Type: application/json' \
  -d '{"end_pose":{"x":<map-x>,"y":<map-y>},"target_speed":2.0}'
```

The request body uses the protobuf JSON field names from
`apollo.external_command.LaneFollowCommand`. The bridge fills the command
header, assigns a command ID when one is not supplied, and forwards the request
to `/apollo/external_command/lane_follow`. Replace `<map-x>` and `<map-y>` with
a destination near a `CITY_DRIVING` lane in the active map. When `heading` is
included, the destination must be within 3 meters of a lane whose heading
differs by at most 1 radian.

Send a valet-parking command:

```shell
curl -X POST http://127.0.0.1:9091/valet_parking \
  -H 'Content-Type: application/json' \
  -d '{"parking_spot_id":"<map-parking-spot-id>","target_speed":2.0}'
```

The valet-parking request uses the protobuf JSON field names from
`apollo.external_command.ValetParkingCommand` and is forwarded to
`/apollo/external_command/valet_parking`.

Use `GET /healthz` to check whether the HTTP process is running. A successful
health check does not guarantee that the Cyber service is available.
