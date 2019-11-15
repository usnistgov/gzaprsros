

#include <ros/ros.h>

//https://github.com/spencer-project/spencer_people_tracking/blob/master/visualization/spencer_tracking_rviz_plugin/src/additional_topic_subscriber.h
#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/ros_topic_property.h>

 rviz::VisualizationManager* manager = vis_frame->getManager();
manager->setFixedFrame("base_link");
