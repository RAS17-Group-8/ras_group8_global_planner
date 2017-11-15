#include <ras_group8_global_planner/GlobalPlanner.hpp>

// only needed for tests
#include <ras_group8_util/BMP.hpp>
#include <cstdio>
//

namespace ras_group8_global_planner {

GlobalPlanner::GlobalPlanner(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  
  get_map_client_=node_handle_.serviceClient<nav_msgs::GetMap>(get_map_service_);
  compute_global_path_server_=node_handle_.advertiseService(compute_path_service_,&GlobalPlanner::computePath,this);
  marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("visualisation_marker", 1,true);

  ROS_INFO("Successfully launched node.");
}

GlobalPlanner::~GlobalPlanner()
{
}

bool GlobalPlanner::readParameters()
{
  if (!node_handle_.getParam("compute_path_service",compute_path_service_))
    return false;
  if (!node_handle_.getParam("get_map_service", get_map_service_))
    return false;
  if (!node_handle_.getParam("map_costs/solid_obstacle", cost_solid_obstacle_))
    return false;
  if (!node_handle_.getParam("map_costs/removable_obstacle", cost_removable_obstacle_))
    return false;
  if (!node_handle_.getParam("map_costs/distance_obstacle", cost_distance_obstacle_))
    return false;
  if (!node_handle_.getParam("map_costs/free_cells", cost_free_cell_))
    return false;
  if (!node_handle_.getParam("obstacle_avoidance/obstacle_boundary", obstacle_boundary_))
    return false;
  if (!node_handle_.getParam("obstacle_avoidance/obstacle_distance", obstacle_distance_))
    return false;
  if (!node_handle_.getParam("size_robot/radius", robot_radius_))
    return false;

  return true;
}

bool GlobalPlanner::readMap()
{
    /*
     * reads the map from the Map server and creates an cost map
     */
    get_map_client_.call(actual_map_);

    width_=actual_map_.response.map.info.width;
    height_=actual_map_.response.map.info.height;
    resolution_=actual_map_.response.map.info.resolution;

    cost_map_.info.width = width_;
    cost_map_.info.height = height_;
    cost_map_.info.resolution = resolution_;
    cost_map_.data.resize(width_ * height_);

    // Create the costmap
    for (int i=0; i<(width_*height_); i++)
    {
        if (actual_map_.response.map.data[i]>obstacle_boundary_)
        {
           GlobalPlanner::mapSmoothing(i);
        }
        else if (cost_map_.data[i]<cost_free_cell_)
        {
            cost_map_.data[i]=cost_free_cell_;
        }
       // ROS_INFO("Map %i  Cost %i",actual_map_.response.map.data[i],cost_map_.data[i]);
    }
    ROS_INFO("Cost Map creation succesfull");

    //////////////////Show Cost Map//////////////////////
//    int res;
//    FILE* f1 = fopen("/home/ras/actual_map.bmp", "wb");

//    if (f1 == NULL) {
//      ROS_ERROR("Failed to open target file");
//    }

//    if (res = ras_group8_util::BMP::write(actual_map_.response.map, f1)) {
//      ROS_ERROR("Failed to write to file (%i)", res);
//    }

//    fclose(f1);

//    FILE* f2 = fopen("/home/ras/cost_map.bmp", "wb");

//    if (f2 == NULL) {
//      ROS_ERROR("Failed to open target file");
//    }
//    res = ras_group8_util::BMP::write(cost_map_, f2);
//    ROS_INFO("Test Pictures are created" );

    ///////////////////////////////////////
}

void GlobalPlanner::mapSmoothing(int cell)
{
   //Adapt obstacels to the size of the robot and creat a gaussian around the obstacle

    int actual_row=lineToRow(cell);
    int actual_column=lineToColumn(cell);
    int cost_value;

    cell_robot_radius_=ceil(robot_radius_/resolution_);
    cell_distance_obstacle_=ceil(obstacle_distance_/resolution_);
    int cell_kernel=cell_robot_radius_+cell_distance_obstacle_;

    int sigma=2*pow(cell_kernel/3,2); // after 3 sigma is the value nearly zero
    int gaus_factor=cost_distance_obstacle_/exp(-(pow(cell_robot_radius_,2))/sigma);
    //value of the direct line from the middlepoint should be the wished value

    for (int y=actual_row-cell_kernel; y<=actual_row+cell_kernel; y++)
    {
        for (int x=actual_column-cell_kernel; x<=actual_column+cell_kernel; x++)
        {
            int position=mapToLine(x,y);

            if (y < 0 || y >= height_ || x < 0 || x >= width_)
            {
            }
            else if (y>=actual_row-cell_robot_radius_ && y<=actual_row+cell_robot_radius_ && x>=actual_column-cell_robot_radius_ && x<=actual_column+cell_robot_radius_)
            {
               //adapt the costmap to the size of the robot
                cost_map_.data[position]=cost_solid_obstacle_;
            }
            else
            {
                //Gaussian Kernel
               cost_value=round(gaus_factor*exp(-(pow(y-actual_row,2)+pow(x-actual_column,2))/sigma));
               if (cost_value>cost_map_.data[position])
               {
                   cost_map_.data[position]=cost_value;
               }
            }
        }
    }
}

bool GlobalPlanner::computePath(nav_msgs::GetPlan::Request &req,
                                nav_msgs::GetPlan::Response &res)
{
    /*
     * Function loads the costmap and calculates the path by the help of the AStar algorithm
     */
    AStarList.clear();
    PathPointList.clear();
    SmoothPathList.clear();

    ROS_INFO("Start to compute path");

    readMap();

    start_x=floor(req.start.pose.position.x/resolution_);
    start_y=floor(req.start.pose.position.y/resolution_);
    goal_x=floor(req.goal.pose.position.x/resolution_);
    goal_y=floor(req.goal.pose.position.y/resolution_);

    int position=mapToLine(start_x,start_y);
    int arrayposition=0;



    if (start_y < 0 || start_y >= height_ || start_x < 0 || start_x >= width_ ||
         goal_y < 0 || goal_y >= height_ || goal_x < 0 || goal_x >= width_ )
    {
        ROS_ERROR("Start or goal lays outside of the map");
        return false;
    }
    if (cost_map_.data[mapToLine(start_x, start_y)]==cost_solid_obstacle_|| cost_map_.data[mapToLine(goal_x, goal_y)]==cost_solid_obstacle_)
    {
        ROS_INFO("Start or goal lays on an obstacle");
    }

    actual_node_.node_x=start_x;
    actual_node_.node_y=start_y;
    actual_node_.come_from_x=-1;
    actual_node_.come_from_y=-1;
    actual_node_.come_from_position=-1;
    actual_node_.actual_cost=cost_map_.data[position];
    actual_node_.estimate_goal_cost=cost_free_cell_*(abs(actual_node_.node_x-goal_x)+abs(actual_node_.node_y-goal_y));
    actual_node_.total_cost=actual_node_.estimate_goal_cost+actual_node_.actual_cost;
    AStarList.push_back(actual_node_);

    lowest_cost_element=0;

    while(AStarList[lowest_cost_element].node_x!=goal_x || AStarList[lowest_cost_element].node_y!=goal_y)
    {
       addOpenNode(AStarList[lowest_cost_element].node_x-1, AStarList[lowest_cost_element].node_y, &AStarList[lowest_cost_element], arrayposition);
       addOpenNode(AStarList[lowest_cost_element].node_x+1, AStarList[lowest_cost_element].node_y, &AStarList[lowest_cost_element], arrayposition);
       addOpenNode(AStarList[lowest_cost_element].node_x, AStarList[lowest_cost_element].node_y-1, &AStarList[lowest_cost_element], arrayposition);
       addOpenNode(AStarList[lowest_cost_element].node_x, AStarList[lowest_cost_element].node_y+1, &AStarList[lowest_cost_element], arrayposition);

       //ROS_INFO("Info x %i y%i %i", AStarList[lowest_cost_element].node_x, AStarList[lowest_cost_element].node_y,AStarList.size());

       AStarList[lowest_cost_element].closed=true; //Add the node to the close loop list

       //Find the node with the lowest total cost in the open node list
       int lowestcost=1000000;
       bool no_path=true;
       arrayposition=0;
       for (int i=0; i<AStarList.size();i++)
       {
           if (!AStarList[i].closed && AStarList[i].total_cost<lowestcost)
           {
               lowestcost=AStarList[i].total_cost;
               arrayposition=i;
               no_path=false;
           }
       }
       lowest_cost_element=arrayposition;
       if (no_path)
       {
           ROS_ERROR("There exist no path");
           return false;
       }
    }

   ROS_INFO("A Star Algorithm was succesfull");
   ROS_INFO("Path cost %i",AStarList[lowest_cost_element].actual_cost);
   res.plan.header.seq=AStarList[lowest_cost_element].actual_cost;

   //create point chain

   while (AStarList[lowest_cost_element].come_from_position!=-1)
   {
       actual_position_.x=AStarList[lowest_cost_element].node_x;
       actual_position_.y=AStarList[lowest_cost_element].node_y;
       PathPointList.push_back(actual_position_);

       lowest_cost_element=AStarList[lowest_cost_element].come_from_position;

       //////Only for Viszualisation///////////////////
       geometry_msgs::Point p;
       p.x = actual_position_.x*resolution_;
       p.y = actual_position_.y*resolution_;
       p.z = 0;
       points_path_.points.push_back(p);
       ///////////////////////////////////////////////////
   }

   if(!pathSmoothing())
   {
       ROS_ERROR("Fail to smooth path");
   }

   int point_number=SmoothPathList.size();

   res.plan.poses.resize(point_number);

   for (int i=0; i<point_number; i++)
   {
      res.plan.poses[i].pose.position.x=((double)SmoothPathList[point_number-1-i].x)*resolution_;
      res.plan.poses[i].pose.position.y=((double)SmoothPathList[point_number-1-i].y)*resolution_;

      //ROS_INFO("Plan x:%f  y;%f ",res.plan.poses[i].pose.position.x/resolution_,res.plan.poses[i].pose.position.y/resolution_);

      //////Only for Viszualisation///////////////////
      geometry_msgs::Point p;
      p.x = res.plan.poses[i].pose.position.x;
      p.y = res.plan.poses[i].pose.position.y;
      p.z = 0.01;
      points_spath_.points.push_back(p);
      ///////////////////////////////////////////////////
   }

   ROS_INFO("Path creation was succesfull, pathpoints: %i ", point_number);
   pathVisualisation();

   return true;
}

void GlobalPlanner::addOpenNode(int x, int y, AStarNode* lowestcostnode, int lastposition)
{
    int position;

    if (x>=0 && x<width_ && y>=0 && y<height_) //Point is inside the map
    {
        position=mapToLine(x,y);

        actual_node_.node_x=x;
        actual_node_.node_y=y;
        actual_node_.come_from_x=lowestcostnode->node_x;
        actual_node_.come_from_y=lowestcostnode->node_y;
        actual_node_.come_from_position=lastposition;
        actual_node_.actual_cost=lowestcostnode->actual_cost+cost_map_.data[position];
        actual_node_.estimate_goal_cost=cost_free_cell_*(abs(x-goal_x)+abs(y-goal_y));
        actual_node_.total_cost=actual_node_.actual_cost+actual_node_.estimate_goal_cost;
        actual_node_.closed=false;

//        ROS_INFO("x:%i y:%i ,Actual Cost %i Total costs %i",x, y ,actual_node_.actual_cost,actual_node_.total_cost);

        bool element_exist=false;

        for (int i=0; i<AStarList.size();i++)
        {
            if (x==AStarList[i].node_x && y==AStarList[i].node_y)
            {
                if (actual_node_.total_cost<AStarList[i].total_cost) // only if the costs are smaller
                {
                    AStarList[i]=actual_node_;
                }
                i=AStarList.size();
                element_exist=true;
            }
        }

        if (!element_exist) //element not in the list
        {
            AStarList.push_back(actual_node_);
        }
    }
}


bool GlobalPlanner::pathSmoothing()
{
    ROS_INFO("Start Path Smooting");
    bool obstacle=false;
    double slope_x;
    double slope_y;
    int delta_x=0;
    int delta_y=0;
    int cost_element;
    GlobalPlanner::PositionXY last_spath_element=PathPointList[0];

    SmoothPathList.push_back(last_spath_element);

    for (int i=1; i<PathPointList.size(); i++)
    {
        delta_x=abs(PathPointList[i].x-last_spath_element.x);
        delta_y=abs(PathPointList[i].y-last_spath_element.y);
        int sign_x=1;
        int sign_y=1;
        if(PathPointList[i].x<last_spath_element.x) {sign_x=-1;}
        if(PathPointList[i].y<last_spath_element.y) {sign_y=-1;}

        if (delta_x==0||delta_y==0){
            slope_x=0;
            slope_y=0;
        }
        else{
            slope_x=((double)(delta_y*sign_y)/(delta_x*sign_x));
            slope_y=((double)(delta_x*sign_x)/(delta_y*sign_y));
        }

//        ROS_INFO("last El x:%i y:%i act El x:%i y:%i ", last_spath_element.x,last_spath_element.y,PathPointList[i].x,PathPointList[i].y);
//        ROS_INFO("Deltax %i delta y %i slope %f", delta_x,delta_y,slope_x);

        if (delta_x>=delta_y)
        {
            //ROS_INFO("X bigger y");
            int dif_y;
            int dif_y_last=0;
            for(int d_x=0; d_x< delta_x ;d_x++)
            {
                dif_y=sign_x*floor(d_x*slope_x);
                cost_element=mapToLine((sign_x*d_x+last_spath_element.x), (dif_y+last_spath_element.y));

                //ROS_INFO("Check element x:%i y:%i cost:%i",(sign_x*d_x+last_spath_element.x), (dif_y+last_spath_element.y),cost_map_.data[cost_element]);

                if (cost_map_.data[cost_element]>cost_free_cell_)
                {
                    obstacle=true;
                    d_x=delta_x;
                }

                if (dif_y!=dif_y_last)
                {
                    cost_element=mapToLine((sign_x*d_x+last_spath_element.x-1), (dif_y+last_spath_element.y));
                    if (cost_map_.data[cost_element]>cost_free_cell_)
                    {
                        obstacle=true;
                        d_x=delta_x;
                    }
                }
                dif_y_last=dif_y;
            }
        }
        else
        {
            //ROS_INFO("y bigger x");
            int dif_x;  //with sign
            int dif_x_last=0;
            for(int d_y=0; d_y< delta_y ;d_y++)
            {
                dif_x=sign_y*floor(d_y*slope_y);
                cost_element=mapToLine((dif_x+last_spath_element.x), (d_y*sign_y+last_spath_element.y));

                //ROS_INFO("Check element x:%i y:%i cost:%i",(dif_x+last_spath_element.x), (d_y*sign_y+last_spath_element.y),cost_map_.data[cost_element]);

                if (cost_map_.data[cost_element]>cost_free_cell_)
                {
                    obstacle=true;
                    d_y=delta_y;
                }

                if (dif_x!=dif_x_last)
                {
                    cost_element=mapToLine((dif_x+last_spath_element.x), (sign_y*d_y+last_spath_element.y-1));
                    if (cost_map_.data[cost_element]>cost_free_cell_)
                    {
                        obstacle=true;
                        d_y=delta_y;
                    }
                }
                dif_x_last=dif_x;
            }

        }


        if (obstacle)
        {
            last_spath_element=PathPointList[i-1];
            SmoothPathList.push_back(last_spath_element);
            obstacle=false;
        }
    }
    return true;
}

void GlobalPlanner::pathVisualisation()
{

    points_path_.header.frame_id = "/map";
    points_path_.header.stamp = ros::Time();
    points_path_.ns="map";
    points_path_.action=visualization_msgs::Marker::ADD;
    points_path_.pose.orientation.w =1.0;
    points_path_.id=0;
    points_path_.type = visualization_msgs::Marker::POINTS;
    points_path_.scale.x = resolution_;
    points_path_.scale.y = resolution_;
    points_path_.color.b = 1.0;
    points_path_.color.a = 1.0;
    marker_pub_.publish(points_path_);

    points_spath_.header.frame_id = "/map";
    points_spath_.header.stamp = ros::Time();
    points_spath_.ns="map";
    points_spath_.action=visualization_msgs::Marker::ADD;
    points_spath_.pose.orientation.w =1.0;
    points_spath_.id=1;
    points_spath_.type = visualization_msgs::Marker::POINTS;
    points_spath_.scale.x = resolution_;
    points_spath_.scale.y = resolution_;
    points_spath_.color.g = 1.0;
    points_spath_.color.a = 1.0;
    marker_pub_.publish(points_spath_);


    ROS_INFO("Vizualisation was succesfull");

}

//The map data is stored in row-major order, starting at (0,0). Example:
//  0:  M[0][0]
//  1:  M[0][1]
//  2:  M[1][0]
//  ...

int GlobalPlanner::mapToLine(int col, int row)
{
  // Check that row and col are in the map
  if (col < 0 || col >= width_ || row < 0 || row >= height_)
  {
    return -1;
  }
  return row * width_ + col;
}

int GlobalPlanner::lineToColumn(int element)
{
    if (element < 0 || element>=(width_*height_))
    {
      return -1;
    }
    return element % width_;
}

int GlobalPlanner::lineToRow(int element)
{
    if (element < 0 || element>=(width_*height_))
    {
      return -1;
    }
    return floor(((double)element)/width_);
}

} /* namespace */
