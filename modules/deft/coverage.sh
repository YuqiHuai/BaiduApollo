source /apollo/scripts/apollo_base.sh
bazel coverage -s --combined_report=lcov --instrumentation_filter="^//modules/planning[/:]" //modules/deft:main_test
rm -rf /home/$USER/deft/genhtml
genhtml --output /home/$USER/deft/genhtml "$(bazel info output_path)/_coverage/_coverage_report.dat"