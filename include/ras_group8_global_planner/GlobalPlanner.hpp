#pragma once

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <vector>
//messages
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
//services
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

using std::vector;

namespace ras_group8_global_planner {

class GlobalPlanner
{
public:
  GlobalPlanner(ros::NodeHandle& node_handle);
  virtual ~GlobalPlanner();

private:

  /* Structures
   */
  struct AStarNode
  {
      int node_x;
      int node_y;
      int come_from_x;
      int come_from_y;
      int come_from_position;
      int actual_cost;
      int estimate_goal_cost;
      int total_cost;
      bool closed;
  };

  struct PositionXY
  {
      int x;
      int y;
  };

  /* Functions
   */
  bool readParameters();
  bool readMap();
  bool computePath(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res);
  void mapSmoothing(int cell);
  int  mapToLine(int row, int col);
  int  lineToColumn(int element);
  int  lineToRow(int element);
  void addOpenNode(int x, int y, AStarNode* lowestcostnode, int lastposition);
  void pathVisualisation();
  bool pathSmoothing();

  /* ROS Objects
   */
  ros::NodeHandle& node_handle_;

  ros::ServiceClient get_map_client_;
  ros::ServiceServer compute_global_path_server_;

  ros::Publisher marker_pub_;

  /* Parameters
   */
  std::string compute_path_service_;
  std::string get_map_service_;

  nav_msgs::GetMap actual_map_;
  nav_msgs::OccupancyGrid cost_map_;

  visualization_msgs::Marker points_path_;
  visualization_msgs::Marker points_spath_;

  /* Variables
   */
  int cost_solid_obstacle_;
  int cost_removable_obstacle_;
  int cost_distance_obstacle_;
  int cost_free_cell_;

  int obstacle_boundary_;
  double obstacle_distance_;
  double robot_radius_;

  int width_;
  int height_;
  float resolution_;

  int cell_robot_radius_;
  int cell_distance_obstacle_;

  int goal_x;
  int goal_y;
  int start_x;
  int start_y;

  AStarNode actual_node_;
  vector<AStarNode> AStarList;
  int lowest_cost_element;

  vector<PositionXY> PathPointList;
  PositionXY actual_position_;

  vector<PositionXY> SmoothPathList;
};

} /* namespace */
