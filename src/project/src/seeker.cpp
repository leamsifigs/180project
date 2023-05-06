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
//after each costmap update, compare the updated global cost map to the original global cost map


class Points {
  public:
    geometry_msgs::msg::Point point;
    int count = 0;
};

std::vector<Points>* pointCount = new std::vector<Points>;

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

void add_point(geometry_msgs::msg::Point point, std::vector<Points>* list_ptr){

  double distance; 

  if (list_ptr->empty()){
    Points new_point;
    new_point.point.x = point.x;
    new_point.point.y = point.y;
    new_point.point.z = 0;
    new_point.count = 1;

    list_ptr->push_back(new_point);
    return;
  }

//iterate through list_ptr to see if a point is within 7 cell units, if so, count++ for point aleady in list,
  for(auto it = list_ptr->begin(); it != list_ptr->end(); ++it){

    distance = std::sqrt(std::pow(it->point.x - point.x, 2) + std::pow(it->point.y - point.y, 2)); 
    if(distance <= 13.0/20.0){
      it->count++;
      return;
    }else{
      Points new_point;
      new_point.point.x = point.x;
      new_point.point.y = point.y;
      new_point.point.z = 0;
      new_point.count = 1;

      list_ptr->push_back(new_point);
    
      return;
    }

  }

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

int count_cell_neighbors_big_radius(nav_msgs::msg::OccupancyGrid grid, int current_row, int current_column, int width, int height){
  int count = 0;
  for(int i = -3; i < 4; i++){ //for each row in 7x7 grid
    for (int j = -3; j < 4; j++){ //for 7 columns
      if (i == 0 && j == 0) continue; //we dont need to look at the cell itself

      // if( i == -2 && j == -2) continue;//ignore corners
      // if( i == -2 && j == 2) continue;
      // if( i == 2  && j == -2) continue;
      // if( i == 2  && j == 2) continue;

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
  int width = newWallMap.info.width;
  int height = newWallMap.info.height;


  for (int i = 0; i < height; i++){ //for each row

      for (int j = 0; j < width; j++){ //for each column
        
        if(count_cell_neighbors_big_radius(originalCostmap, i,j,width,height) == 0 && count_cell_neighbors(newWallMap, i, j, width, height) >= 3){ //if there are no walls on the original map surrounding this cell, and there are at least 4 new nearby walls, get the coords of this cell


          //need to convert them to world coords

          newWallMap.data[i*width+j] = 2; //for debug reasons, detected pillar walls will be a 2

          pillar_location = std::make_shared<geometry_msgs::msg::Point>();
          pillar_location->z = 0;
          //200,200 is the center pillar's location (the origin of the world)
          // 384 array elements for a row/column, 20 meters for the side length of the map in world coords 384/20 = 19.2 array cells per gazebo meter
          pillar_location->x = (double(j-200))/double(20);  //FIXME trying 12 based on algebraically solving for the conversion rate to get real coordinate location
          pillar_location->y = -(double(i-200))/double(20) - 0.5; //subtract 0.5 because it adds 0.5 for all of them 

          
          std::cout << "Pillar wall at " << pillar_location->x << ", " << pillar_location->y << " (" << i << ", " << j << ")" <<std::endl; //for debug purposes we are printing every wall that could be part of the pillar
          add_point(*pillar_location, pointCount);
        }

      }
    }

    std::cout<< "PRINTING NEWWALLMAP (TWO IS A DETECTED PILLAR)" << std::endl;

    // print_costmap(newWallMap);

    std::cout << std::endl; 

  return pillar_location;
}

void costmapcallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
 

  mostrecentcostmap = *msg;
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
    }
  }
  
  if (!gotFirstCostmap){
    // std::cout << "setting firstcostmap" << std::endl;
    gotFirstCostmap = true;
    firstcostmap = mostrecentcostmap; //if its the first map we ever get, copy it to this
    
    std::cout<< "PRINTING FIRST COST MAP" << std::endl;
    // print_costmap(firstcostmap);
  } else{
    //if not the first, then compare it to the first
     
    // std::cout << "calling costmapcomparison" << std::endl;

    std::cout<< "PRINTING MOST RECENT COST MAP" << std::endl;

    // print_costmap(mostrecentcostmap);

    std::cout << std::endl; 

    nav_msgs::msg::OccupancyGrid newWallMap = costmapcomparison(firstcostmap, mostrecentcostmap);
    // std::cout << "finished costmapcomparison, calling check_for_extra_pillars" << std::endl;
    geometry_msgs::msg::Point::SharedPtr point = check_for_extra_pillars(firstcostmap, newWallMap);
    // std::cout << "finished check_for_extra_pillars" << std::endl;
    
    // print_costmap(mostrecentcostmap);
  }

  
  

}

geometry_msgs::msg::Point most_frequent_point(std::vector<Points>* list_ptr){
  //returns the point that occurs most frequently

  geometry_msgs::msg::Point current_max_point;
  int max_count = 0;

  for (long unsigned int i = 0; i < list_ptr->size(); i++){
    if ( (*list_ptr)[i].count > max_count ){
      max_count = (*list_ptr)[i].count;
      current_max_point = (*list_ptr)[i].point;
    }
  }

  return current_max_point;
}



int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  rclcpp::Node::SharedPtr nodeh = rclcpp::Node::make_shared("seeker");
  auto globalcostmapSub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap",1000,&costmapcallback); //set up subscription for costmaps

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

 //FollowWaypoints
  geometry_msgs::msg::PoseStamped p1,p2,p3,p4,p5,p6,p7,p8;
    p1.pose.position.x = -1;
    p1.pose.position.y = 1.6;

    p2.pose.position.x = 1;
    p2.pose.position.y = 1.6;

    p3.pose.position.x = 1.5;
    p3.pose.position.y = 0.5;
    
    p4.pose.position.x = -0.5;
    p4.pose.position.y = 0.5;
    
    p5.pose.position.x = -0.5;
    p5.pose.position.y = -0.5;
    
    p6.pose.position.x = 2;
    p6.pose.position.y = -0.5;

    p7.pose.position.x = 0.675;
    p7.pose.position.y = -1.75;

    p8.pose.position.x = -1.5;
    p8.pose.position.y = -1.5;

  std::vector<geometry_msgs::msg::PoseStamped> waypointList;
    waypointList.push_back(p1);
    waypointList.push_back(p2);
    waypointList.push_back(p3);
    waypointList.push_back(p4);
    waypointList.push_back(p5);
    waypointList.push_back(p6);
    waypointList.push_back(p7);
    waypointList.push_back(p8);
    navigator.FollowWaypoints(waypointList);
  while ( ! navigator.IsTaskComplete() ) {
    rclcpp::spin_some(nodeh);
  }
  auto result = navigator.GetResult(); 
  if ( result == rclcpp_action::ResultCode::SUCCEEDED ) 
    std::cout << "FollowWaypoints action succeeded" << std::endl;
  else
    std::cout << "FollowWaypoints goal was not achieved" << std::endl;

  // return to origin
  goal_pos->position.x = -2;
  goal_pos->position.y = -0.5;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) {
    rclcpp::spin_some(nodeh);
  }



  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    rclcpp::spin_some(nodeh);
  }

  
  //get most frequent pillar

  std::cout <<" PRINTING ALL POINT CLUSTERS" << std::endl;
  for(long unsigned int i = 0; i < pointCount->size(); i++){
    std::cout << (*pointCount)[i].point.x << ", " << (*pointCount)[i].point.y << " Count: " << (*pointCount)[i].count << std::endl;
  }
  std::cout << std::endl;

  geometry_msgs::msg::Point pillar_location =  most_frequent_point(pointCount);
  std::cout << "Pillar is at: " << pillar_location.x << ", " <<pillar_location.y <<std::endl;

  rclcpp::shutdown(); // shutdown ROS
  return 0;
}