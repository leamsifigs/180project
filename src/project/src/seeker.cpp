/*
Copyright 2023 Stefano Carpin

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <math.h>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//before robot is able to update the cost map at all, store a copy of the global cost map
//after each laser scan, compare the updated local cost map to the original global cost map

//or try this

//copy the map to a variable
//after each laser scan, if there are x lasers that are detecting a wall that is not on the map, that is likely a pillar

//do math to find the location


nav_msgs::msg::OccupancyGrid map;
std::vector<int8_t, std::allocator<int8_t>> mapVector;
bool mapReceived = false;

void mapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){

  if (mapReceived) return;

  mapReceived = true;
  mapVector = msg->data;
  int width = msg->info.width;
  int height = msg->info.height;

  for (int i = 0; i < height; i++){ //for each row

    for (int j = 0; j < width; j++){ //for each column
      std::cout << int(msg->data[i*width+j]) << " ";
    }
    std::cout << std::endl;
  }
  return;
}

float angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max;
// float ranges[];
// float intensities[];

void lasercallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

  // Angle Min: 0
  // Angle Max: 6.28
  // Angle Increment: 0.017493
  // Time Increment: 0
  // Scan Time: 0
  // Range Min: 0.12
  // Range Max: 3.5

  angle_min = msg->angle_min;
  angle_max = msg->angle_max;
  angle_increment = msg->angle_increment;
  time_increment = msg->time_increment;
  scan_time = msg->scan_time;
  range_min = msg-> range_min;
  range_max = msg->range_max;

  // std::cout << "Angle Min: " << angle_min << std::endl;
  // std::cout << "Angle Max: " << angle_max << std::endl;
  // std::cout << "Angle Increment: " << angle_increment << std::endl;
  // std::cout << "Time Increment: " << time_increment << std::endl;
  // std::cout << "Scan Time: " << scan_time << std::endl;
  // std::cout << "Range Min: " << range_min << std::endl;
  // std::cout << "Range Max: " << range_max << std::endl;
  // std::cout << std::endl;

  for (auto it = msg->ranges.begin(); it != msg->ranges.end(); ++it){ //iterate through each laser reading
    //first, we need to find the x and y coords for each laser 
    if(!std::isinf(*it)){ // if the reading is not infinity
      // std::cout << *it << ", ";

      //chat gpt code

      // std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      // std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      // geometry_msgs::msg::Point laser_point;
      // laser_point.x = msg->ranges[0] * std::cos(msg->angle_min);
      // laser_point.y = msg->ranges[0] * std::sin(msg->angle_min);
      // laser_point.z = 0.0;

      // // Transform the laser scan data from the sensor frame to the base frame of the robot
      // geometry_msgs::msg::Transform sensor_to_base_transform = tf_buffer_->lookupTransform(
      //   "base_link", msg->header.frame_id, laser_point.header.stamp, rclcpp::Duration::from_seconds(0.1));

      // tf2::doTransform(laser_point, laser_point, sensor_to_base_transform);

      // // Transform the laser scan data from the base frame of the robot to the world frame
      // geometry_msgs::msg::Transform base_to_world_transform = tf_buffer_->lookupTransform(
      //   "world", "base_link", laser_point.header.stamp, rclcpp::Duration::from_seconds(0.1));

      // tf2::doTransform(laser_point, laser_point, base_to_world_transform);

      // // Extract the position where the laser landed in the world frame
      // RCLCPP_INFO(this->get_logger(), "Laser landed at (%f, %f, %f) in world frame",
      //             laser_point.point.x, laser_point.point.y, laser_point.point.z);




    }


    // std::cout << *it << ", " << std::endl;
  }

  // std::cout << std::endl;


}

nav_msgs::msg::OccupancyGrid firstcostmap;
nav_msgs::msg::OccupancyGrid mostrecentcostmap;
bool gotFirstCostmap = false;

nav_msgs::msg::OccupancyGrid costmapcomparison(nav_msgs::msg::OccupancyGrid firstmap, nav_msgs::msg::OccupancyGrid recentmap){
  // std::cout << "initializing comparisoncostmap" << std::endl;
  nav_msgs::msg::OccupancyGrid comparisoncostmap = recentmap;
  // std::cout << "initialized comparisoncostmap" << std::endl;
  int width = recentmap.info.width;
  int height = recentmap.info.height;

  for (int i = 0; i < height; i++){ //for each row

      for (int j = 0; j < width; j++){ //for each column
        if(recentmap.data[i*width+j] == 1 && firstmap.data[i*width+j] == 0){
          // std::cout << "if condition works" << std::endl;
          comparisoncostmap.data[i*width+j] = 1;
          // std::cout << "updating comparisoncostmap works" << std::endl;
        } else{
          // std::cout << "else condition works" << std::endl;
          comparisoncostmap.data[i*width+j] = 0;
          // std::cout << "updating comparisoncostmap works" << std::endl;
        }

      }
  }
  // std::cout << "costmapcomparison() done! returning the map" << std::endl;
  return comparisoncostmap;

  
}  

void print_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap){

  int width = costmap->info.width;
  int height = costmap->info.height;

  for (int i = 0; i < height; i++){ //for each row

    for (int j = 0; j < width; j++){ //for each column
      std::cout << int(costmap->data[i*width+j]) << " ";
    }

    std::cout << std::endl;
  }

  return;
}

void print_costmap(const nav_msgs::msg::OccupancyGrid costmap){

  int width = costmap.info.width;
  int height = costmap.info.height;

  for (int i = 0; i < height; i++){ //for each row

    for (int j = 0; j < width; j++){ //for each column
      std::cout << int(costmap.data[i*width+j]) << " ";
    }

    std::cout << std::endl;
  }

  return;
}




int count_cell_neighbors(nav_msgs::msg::OccupancyGrid::SharedPtr grid, int current_row, int current_column, int width, int height){
  int count = 0;
  for(int i = -1; i < 2; i++){ //for each row in 3x3 grid
    for (int j = -1; j < 2; j++){ //for 3 columns
      if (i == 0 && j == 0) continue; //we dont need to look at the cell itself

      int row = current_row + i;
      int column = current_column + j;
      if (row < 0 || row >= height || column < 0 || column >= width) continue; //this would be out of range, skip to avoid segfault

      if (grid->data[row*width+column] == 1 || grid->data[row*width+column] == 2) count++; //count cell if it has a wall

    }

  }

  return count;
}

int count_cell_neighbors(nav_msgs::msg::OccupancyGrid grid, int current_row, int current_column, int width, int height){
  int count = 0;
  for(int i = -1; i < 2; i++){ //for each row in 3x3 grid
    for (int j = -1; j < 2; j++){ //for 3 columns
      if (i == 0 && j == 0) continue; //we dont need to look at the cell itself

      int row = current_row + i;
      int column = current_column + j;
      if (row < 0 || row >= height || column < 0 || column >= width) continue; //this would be out of range, skip to avoid segfault

      if (grid.data[row*width+column] == 1 || grid.data[row*width+column] == 2) count++; //count cell if it has a wall

    }

  }

  return count;
}

geometry_msgs::msg::Point::SharedPtr check_for_extra_pillars(nav_msgs::msg::OccupancyGrid originalCostmap, nav_msgs::msg::OccupancyGrid newWallMap){ //only pass in a map that has ONLY walls that are not detected in the original costmap
  //returns coords of the pillar if found, otherwise returns nullptr

  

  geometry_msgs::msg::Point::SharedPtr pillar_location = nullptr;
  // bool pillar_found = false;
  int width = newWallMap.info.width;
  int height = newWallMap.info.height;

  //potentially add a check for whether a new wall is one space next to an original wall (could potentially be the wall shifting over due to error)

  for (int i = 0; i < height; i++){ //for each row

      for (int j = 0; j < width; j++){ //for each column
        
        if(count_cell_neighbors(originalCostmap, i,j,width,height) == 0 && count_cell_neighbors(newWallMap, i, j, width, height) >= 4){ //if there are no walls on the original map surrounding this cell, and there are at least 4 new nearby walls, get the coords of this cell

          

          //need to convert them to world coords
          // pillar_location = new geometry_msgs::msg::Point();

          newWallMap.data[i*width+j] = 2; //for debug reasons, detected pillar walls will be a 2

          pillar_location = std::make_shared<geometry_msgs::msg::Point>();
          pillar_location->z = 0;
          //200,200 is the center pillar's location (the origin of the world)
          // 384 array elements for a row/column, 20 meters for the side length of the map in world coords 384/20 = 19.2 array cells per gazebo meter
          pillar_location->x = (j-200)/12;  //FIXME trying 12 based on algebraically solving for the conversion rate to get real coordinate location
          pillar_location->y = -(i-200)/12;

          std::cout << "Pillar wall at " << pillar_location->x << ", " << pillar_location->y << " (" << i << ", " << j << ")" <<std::endl; //for debug purposes we are printing every wall that could be part of the pillar
        }

      }
    }

    std::cout<< "PRINTING NEWWALLMAP (TWO IS A DETECTED PILLAR)" << std::endl;

    print_costmap(newWallMap);

    std::cout << std::endl; 

  return pillar_location;
}

void costmapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  
  // try{
  //   delete mostrecentcostmap; //prevent memory leaks :)
  // }

  mostrecentcostmap = *msg; //this could be wrong
  // std::cout << "assigned mostrecentcostmap to message" << std::endl;

  int width = msg->info.width;
  int height = msg->info.height;

  for (int i = 0; i < height; i++){ //for each row

    for (int j = 0; j < width; j++){ //for each column  //map gets vertically inverted here and also we only care about walls
      if(msg->data[i*width+j] == 100){
        mostrecentcostmap.data[(height-1-i)*width+j] = 1;
      } else{
        mostrecentcostmap.data[(height-1-i)*width+j] = 0;
      }
      // std::cout << int(firstcostmap.data[i*width+j]) << " ";
    }
    // std::cout << std::endl;
  }


  
  if (!gotFirstCostmap){
    // std::cout << "setting firstcostmap" << std::endl;
    gotFirstCostmap = true;
    firstcostmap = mostrecentcostmap; //if its the first map we ever get, copy it to this
    
    std::cout<< "PRINTING FIRST COST MAP" << std::endl;
    print_costmap(firstcostmap);
  } else{
    //if not the first, then compare it to the first
     
    // nav_msgs::msg::OccupancyGrid::SharedPtr newWallmap = mostrecentcostmap;
    // std::cout << "calling costmapcomparison" << std::endl;

    std::cout<< "PRINTING MOST RECENT COST MAP" << std::endl;

    print_costmap(mostrecentcostmap);

    std::cout << std::endl; 

    nav_msgs::msg::OccupancyGrid newWallMap = costmapcomparison(firstcostmap, mostrecentcostmap);
    // std::cout << "finished costmapcomparison, calling check_for_extra_pillars" << std::endl;
    geometry_msgs::msg::Point::SharedPtr point = check_for_extra_pillars(firstcostmap, newWallMap);
    // std::cout << "finished check_for_extra_pillars" << std::endl;
    
    // print_costmap(mostrecentcostmap);
  }

  
  

}

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  rclcpp::Node::SharedPtr nodeh = rclcpp::Node::make_shared("seeker");
  // auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",1000,&mapcallback);
  // auto laserSub = nodeh->create_subscription<sensor_msgs::msg::LaserScan>("/scan",1000,&lasercallback); //set up subscription for laser
  auto globalcostmapSub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",1000,&costmapcallback); //set up subscription for costmaps
  // rclcpp::spin(nodeh);


  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
    // rclcpp::spin_some(nodeh);
  }

  //create goal pose
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;


  goal_pos->position.x = -0.5;  //move robot to face sample pillar
  goal_pos->position.y = -1.5;
  goal_pos->orientation.w = -1;

  // move to new pose
  navigator.GoToPose(goal_pos);

  //the while loops are so the code waits for the robot to complete its task
  while ( ! navigator.IsTaskComplete() ) {
    rclcpp::spin_some(nodeh);
  }

  // gotFirstCostmap = false;
  // costmapcallback(mostrecentcostmap);

  // nav_msgs::msg::OccupancyGrid tempcostmap = mostrecentcostmap;
  // nav_msgs::msg::OccupancyGrid::SharedPtr costmap_ptr();
  


 // test FollowWaypoints
  geometry_msgs::msg::PoseStamped p1,p2,p3;
    p1.pose.position.x = -2;
    p1.pose.position.y = -1;
    p1.pose.orientation.w = 2;

    p2.pose.position.x = -2;
    p2.pose.position.y = 1;

    p3.pose.position.x = 2;
    p3.pose.position.y = 1;


  std::vector<geometry_msgs::msg::PoseStamped> pointList;
    pointList.push_back(p1);
    pointList.push_back(p2);
    pointList.push_back(p3);
    navigator.FollowWaypoints(pointList);
  while ( ! navigator.IsTaskComplete() ) {
    // rclcpp::spin_some(nodeh);
  }
  auto result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED ) 
    std::cout << "FollowWaypoints action succeeded" << std::endl;
  else
    std::cout << "FollowWaypoints goal was not achieved" << std::endl;

  goal_pos->position.x = -2;
  goal_pos->position.y = -0.5;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
  
  }


  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }



  // complete here....
  

  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
