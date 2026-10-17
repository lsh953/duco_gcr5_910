cmake_minimum_required(VERSION 3.22)
project(duco_gcr5_910_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/send_joint_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # The tests import rclpy, so they must run with the interpreter that belongs
  # to the ROS 2 distribution.  The first "python3" on PATH is not necessarily
  # that interpreter (a conda or virtualenv installation can shadow the system
  # one), and ament_add_pytest_test would then silently register nothing, so
  # the candidates are probed instead of assumed.
  find_package(Python3 QUIET COMPONENTS Interpreter)
  set(_test_python "")
  foreach(_candidate "${Python3_EXECUTABLE}" "/usr/bin/python3")
    if(_test_python STREQUAL "" AND _candidate AND EXISTS "${_candidate}")
      execute_process(
        COMMAND "${_candidate}" -c "import rclpy, pytest, control_msgs, action_msgs"
        RESULT_VARIABLE _probe_result
        OUTPUT_QUIET
        ERROR_QUIET)
      if(_probe_result EQUAL 0)
        set(_test_python "${_candidate}")
      endif()
    endif()
  endforeach()

  if(_test_python STREQUAL "")
    message(WARNING
      "No Python interpreter with rclpy, pytest, control_msgs and action_msgs was "
      "found, so test_send_joint_waypoints.py is NOT registered and NOT executed.")
  else()
    message(STATUS "send_joint_waypoints.py tests run with ${_test_python}")
    ament_add_pytest_test(test_send_joint_waypoints test/test_send_joint_waypoints.py
      PYTHON_EXECUTABLE "${_test_python}"
      TIMEOUT 600
    )
  endif()
endif()

