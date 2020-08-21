add_executable(timmo_cloud_node cloud_node.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timmo_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timmo_cloud_node timmo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timmo_cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(timmo_cloud_nodelet cloud_nodelet.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timmo_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timmo_cloud_nodelet timmo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timmo_cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(timmo_transform_node transform_node.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timmo_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timmo_transform_node timmo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timmo_transform_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(timmo_transform_nodelet transform_nodelet.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timmo_transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timmo_transform_nodelet timmo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timmo_transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
