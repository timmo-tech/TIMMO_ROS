# build the driver node
add_executable(timmo_node timmo_node.cc driver.cc)
add_dependencies(timmo_node timmo_driver_gencfg)
target_link_libraries(timmo_node
  timmo_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(timmo_driver_nodelet nodelet.cc driver.cc)
add_dependencies(timmo_driver_nodelet timmo_driver_gencfg)
target_link_libraries(timmo_driver_nodelet
  timmo_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS timmo_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS timmo_driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
