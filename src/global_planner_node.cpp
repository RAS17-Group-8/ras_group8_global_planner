#include <ros/ros.h>
#include <ras_group8_global_planner/GlobalPlanner.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ras_group8_global_planner");
  ros::NodeHandle node_handle("~");

  ras_group8_global_planner::GlobalPlanner main_object(node_handle);

  ros::spin();
  return 0;
}
