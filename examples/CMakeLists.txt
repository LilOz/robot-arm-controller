cmake_minimum_required(VERSION 3.15)

add_executable(fk_example fk_example.cpp)
add_executable(ik6_example ik6_example.cpp)
add_executable(cart_traj_example cart_traj_example.cpp)
add_executable(control_server control_server.cpp)

foreach(example
  fk_example
  ik6_example
  cart_traj_example
  control_server
)
  target_link_libraries(${example} PRIVATE robot_core)
endforeach()
