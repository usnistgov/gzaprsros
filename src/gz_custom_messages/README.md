Catkin_make Protobuf in ROS
======================

catkin_make ROS build of Gazebo custom messages. Borrowed from
https://answers.ros.org/question/192882/using-catkin-to-generate-and-use-protobuf-messages/

where python protoc was removed and more importantly CMakeLists.txt command 

	find_package(ProtocolBuffers REQUIRED)

was changed to

	find_package(Protobuf REQUIRED)

I changed the install destination to include folder under current package folder. This can be a problem as include should automatically be cleaned but wasn't.
