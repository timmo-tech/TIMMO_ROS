add_library(timmo_input input.cc)
target_link_libraries(timmo_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(timmo_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS timmo_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
