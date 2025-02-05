source /apollo/scripts/apollo_base.sh
bazel coverage -s --combined_report=lcov --instrumentation_filter="^//modules/planning[/:]" //modules/deft:main_test
genhtml --output /apollo/modules/deft/genhtml "$(bazel info output_path)/_coverage/_coverage_report.dat"