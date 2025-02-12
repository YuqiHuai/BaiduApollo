#include <iostream>
#include "cyber/common/global_data.h"
#include "cyber/init.h"
#include <memory>
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/sim_control_standalone/sim_control.h"
#include <unistd.h>
#include <stdlib.h>

// bazel build //modules/sim_control_standalone:sim_control_standalone
// bazel run //modules/sim_control_standalone:sim_control_standalone
// ./bazel-bin/modules/sim_control_standalone/sim_control_standalone

// nohup /apollo/bazel-bin/modules/sim_control/sim_control_main &
// ps -ef | grep "sim_control_main" | grep -v 'grep' | awk '{print $2}' | xargs kill -9

int float_test(const char *s) {
    char *endptr;
    errno = 0;
    strtof(s, &endptr);
    if (s == endptr) {
        return 0;
    }
    while (isspace((unsigned char) *endptr)) {
        // look past the number for junk
        endptr++;
    }
    if (*endptr) {
        return 0;
    }
    if (errno) {
        // likely under/overflow
    return 0;
    }

    return 1;
}

int main(int argc, char *argv[]) {
    apollo::cyber::Init(argv[0]);

    google::ReadFromFlagsFile("/apollo/modules/common/data/global_flagfile.txt", argv[0], true);
    std::unique_ptr<apollo::dreamview::MapService> map_service_;
    std::unique_ptr<apollo::dreamview::SimControl> sim_control_;

    map_service_.reset(new apollo::dreamview::MapService());
    sim_control_.reset(new apollo::dreamview::SimControl(map_service_.get()));

    if (argc == 4 && float_test(argv[1]) && float_test(argv[2]) && float_test(argv[3])) {
        double x = atof(argv[1]);
        double y = atof(argv[2]);
        double heading = atof(argv[3]);
        sim_control_->Start(x, y, heading);
    } else if (argc == 3 && float_test(argv[1]) && float_test(argv[2])) {
        double x = atof(argv[1]);
        double y = atof(argv[2]);
        sim_control_->Start(x, y);
    } else {
        sim_control_->Start();
    }

    apollo::cyber::WaitForShutdown();
    return 0;
}