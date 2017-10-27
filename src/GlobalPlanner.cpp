#include <ras_group8_global_planner/GlobalPlanner.hpp>

// STD
#include <string>

namespace ras_group8_global_planner {

GlobalPlanner::GlobalPlanner(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  subscriber_ = node_handle_.subscribe(subscriber_topic_, 1,
                                      &GlobalPlanner::topicCallback, this);

  ROS_INFO("Successfully launched node.");
}

GlobalPlanner::~GlobalPlanner()
{
}

bool GlobalPlanner::readParameters()
{
  if (!node_handle_.getParam("subscriber_topic", subscriber_topic_))
    return false;
  return true;
}

void GlobalPlanner::topicCallback(const phidgets::motor_encoder& msg)
{
}


} /* namespace */
