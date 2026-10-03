cmake_minimum_required(VERSION 3.8)
project(xarm_gazebo_driver_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/wait_for_feedback DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(robot_description test/test_robot_description.py)
  ament_add_pytest_test(simulation_namespace test/test_simulation_namespace.py
    TIMEOUT 120
  )
  ament_add_pytest_test(simulation_bringup test/test_simulation_bringup.py
    TIMEOUT 180
  )
endif()

ament_package()
