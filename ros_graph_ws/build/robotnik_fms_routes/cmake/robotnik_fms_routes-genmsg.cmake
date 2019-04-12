# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(WARNING "Invoking generate_messages() without having added any message or service file before.
You should either add add_message_files() and/or add_service_files() calls or remove the invocation of generate_messages().")
message(STATUS "robotnik_fms_routes: 0 messages, 0 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Irobotnik_msgs:/home/jose/catkin_ws/telefonica_ws/src/demo_telefonica_rb1/robotnik_msgs/msg;-Irobotnik_msgs:/home/jose/catkin_ws/telefonica_ws/devel/share/robotnik_msgs/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Irobot_local_control_msgs:/home/jose/catkin_ws/telefonica_ws/src/demo_telefonica_rb1/robot_local_control_msgs/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Iprocedures_msgs:/home/jose/catkin_ws/telefonica_ws/src/demo_telefonica_rb1/procedures_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotnik_fms_routes_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(robotnik_fms_routes
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotnik_fms_routes
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotnik_fms_routes_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotnik_fms_routes_generate_messages robotnik_fms_routes_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(robotnik_fms_routes_gencpp)
add_dependencies(robotnik_fms_routes_gencpp robotnik_fms_routes_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotnik_fms_routes_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(robotnik_fms_routes
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotnik_fms_routes
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotnik_fms_routes_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotnik_fms_routes_generate_messages robotnik_fms_routes_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(robotnik_fms_routes_geneus)
add_dependencies(robotnik_fms_routes_geneus robotnik_fms_routes_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotnik_fms_routes_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(robotnik_fms_routes
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotnik_fms_routes
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotnik_fms_routes_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotnik_fms_routes_generate_messages robotnik_fms_routes_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(robotnik_fms_routes_genlisp)
add_dependencies(robotnik_fms_routes_genlisp robotnik_fms_routes_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotnik_fms_routes_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(robotnik_fms_routes
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotnik_fms_routes
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotnik_fms_routes_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotnik_fms_routes_generate_messages robotnik_fms_routes_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(robotnik_fms_routes_gennodejs)
add_dependencies(robotnik_fms_routes_gennodejs robotnik_fms_routes_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotnik_fms_routes_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(robotnik_fms_routes
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotnik_fms_routes
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotnik_fms_routes_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotnik_fms_routes_generate_messages robotnik_fms_routes_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(robotnik_fms_routes_genpy)
add_dependencies(robotnik_fms_routes_genpy robotnik_fms_routes_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotnik_fms_routes_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotnik_fms_routes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotnik_fms_routes
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotnik_fms_routes_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET robotnik_msgs_generate_messages_cpp)
  add_dependencies(robotnik_fms_routes_generate_messages_cpp robotnik_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robotnik_fms_routes_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET robot_local_control_msgs_generate_messages_cpp)
  add_dependencies(robotnik_fms_routes_generate_messages_cpp robot_local_control_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotnik_fms_routes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotnik_fms_routes
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotnik_fms_routes_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET robotnik_msgs_generate_messages_eus)
  add_dependencies(robotnik_fms_routes_generate_messages_eus robotnik_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(robotnik_fms_routes_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET robot_local_control_msgs_generate_messages_eus)
  add_dependencies(robotnik_fms_routes_generate_messages_eus robot_local_control_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotnik_fms_routes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotnik_fms_routes
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotnik_fms_routes_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET robotnik_msgs_generate_messages_lisp)
  add_dependencies(robotnik_fms_routes_generate_messages_lisp robotnik_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robotnik_fms_routes_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET robot_local_control_msgs_generate_messages_lisp)
  add_dependencies(robotnik_fms_routes_generate_messages_lisp robot_local_control_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotnik_fms_routes)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotnik_fms_routes
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotnik_fms_routes_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET robotnik_msgs_generate_messages_nodejs)
  add_dependencies(robotnik_fms_routes_generate_messages_nodejs robotnik_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(robotnik_fms_routes_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET robot_local_control_msgs_generate_messages_nodejs)
  add_dependencies(robotnik_fms_routes_generate_messages_nodejs robot_local_control_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotnik_fms_routes)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotnik_fms_routes\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotnik_fms_routes
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotnik_fms_routes_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET robotnik_msgs_generate_messages_py)
  add_dependencies(robotnik_fms_routes_generate_messages_py robotnik_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robotnik_fms_routes_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET robot_local_control_msgs_generate_messages_py)
  add_dependencies(robotnik_fms_routes_generate_messages_py robot_local_control_msgs_generate_messages_py)
endif()
