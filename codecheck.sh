rm -r build
rm -r devel
mkdir reports
catkin config --cmake-args -DCMAKE_C_FLAGS=-fprofile-arcs -DCMAKE_C_FLAGS=-ftest-coverage -DCMAKE_EXE_LINKER_FLAGS=-fprofile-arcs -DCMAKE_EXE_LINKER_FLAGS=-ftest-coverage -DCMAKE_BUILD_TYPE=Coverage
catkin build szelectricity_common szelectricity_msgs
catkin build
catkin run_tests
#gcovr -r ./src/szelectricity_gazebo_plugins/src/ ./build -x -o  reports/gcovr-report.gazebo_plugins.xml
#gcovr -r ./src/szelectricity_gazebo_plugins/src/ ./build -b -x -o reports/gcovr-report.gazebo_plugins.b.xml

#gcovr -r ./src/szelectricity_control/src ./build -x -o  reports/gcovr-report.gazebo_plugins.xml
#gcovr -r ./src/szelectricity_control/src ./build -b -x -o reports/gcovr-report.gazebo_plugins.b.xml

#gcovr -r ./src/szelectricity_common/include/szelectricity_common ./build -x -o  reports/gcovr-report.szelectricity_common.xml
#gcovr -r ./src/szelectricity_common/include/szelectricity_common ./build -b -x -o reports/gcovr-report.szelectricity_common.b.xml

#gcovr -r ./src/szelectricity_odometry/src/*.cpp ./src/szelectricity_odometry/include/szelectricity_odometry/*.hpp ./build -x -o  reports/gcovr-report.szelectricity_common.xml
#gcovr -r ./src/szelectricity_odometry/src/*.cpp ./src/szelectricity_odometry/include/szelectricity_odometry/*.hpp ./build -b -x -o reports/gcovr-report.szelectricity_common.b.xml
gcovr --filter src/szelectricity_common --exclude src/szelectricity_common/test -x -o reports/gcovr-report.szelectricity_common.xml
gcovr --filter src/szelectricity_common --exclude src/szelectricity_common/test -b -x -o reports/gcovr-report.szelectricity_common.xml
gcovr --filter src/motor_driver --exclude src/motor_driver/test -x -o reports/gcovr-report.motor_driver.xml
gcovr --filter src/motor_driver --exclude src/motor_driver/test -b -x -o reports/gcovr-report.motor_driver.xml
gcovr --filter src/szelectricity_teleop --exclude src/szelectricity_teleop/test -x -o reports/gcovr-report.szelectricity_teleop.xml
gcovr --filter src/szelectricity_teleop --exclude src/szelectricity_teleop/test -b -x -o reports/gcovr-report.szelectricity_teleop.xml
gcovr --filter src/szenergy_config --exclude src/szenergy_config/test -x -o reports/gcovr-report.szenergy_config.xml
gcovr --filter src/szenergy_config --exclude src/szenergy_config/test -b -x -o reports/gcovr-report.szenergy_config.xml

# Engine control cppcheck
cppcheck --xml --xml-version=2 --enable=all src/motor_driver/src/* 2> reports/cppcheck-report.szelectricity_common.xml
cppcheck-htmlreport --source-dir=. --report-dir=reports/html/szelectricity_common --file=reports/cppcheck-report.szelectricity_common.xml
# Szelectricity control cppcheck
cppcheck --xml --xml-version=2 --enable=all src/tftree/src/*.cpp 2> reports/cppcheck-report.szelectricity_control.xml
cppcheck-htmlreport --source-dir=. --report-dir=reports/html/szelectricity_control --file=reports/cppcheck-report.szelectricity_control.xml
# Szelectricity teleop cppcheck
cppcheck --xml --xml-version=2 --enable=all src/szelectricity_teleop/src/*.cpp 2> reports/cppcheck-report.szelectricity_control.xml
cppcheck-htmlreport --source-dir=. --report-dir=reports/html/szelectricity_control --file=reports/cppcheck-report.szelectricity_control.xml

#firefox reports/html/gazebo_plugin/index.html
# Scanner
~/sonar-scanner-3.2.0.1227-linux/bin/sonar-scanner
rm -r build
rm -r devel
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build szelectricity_msgs szelectricity_common
catkin build
catkin run_tests