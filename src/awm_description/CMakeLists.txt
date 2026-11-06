cmake_minimum_required(VERSION 3.16)
project(awm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py TIMEOUT 120)
  ament_add_pytest_test(test_cell_v2 test/test_cell_v2.py TIMEOUT 120)
  ament_add_pytest_test(test_head_hcad test/test_head_hcad.py TIMEOUT 300)
endif()

ament_package()
