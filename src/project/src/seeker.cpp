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

  std::cout << "Angle Min: " << angle_min << std::endl;
  std::cout << "Angle Max: " << angle_max << std::endl;
  std::cout << "Angle Increment: " << angle_increment << std::endl;
  std::cout << "Time Increment: " << time_increment << std::endl;
  std::cout << "Scan Time: " << scan_time << std::endl;
  std::cout << "Range Min: " << range_min << std::endl;
  std::cout << "Range Max: " << range_max << std::endl;

  std::cout << std::endl;


}

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  rclcpp::Node::SharedPtr nodeh = rclcpp::Node::make_shared("seeker");
  auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",1000,&mapcallback);
  auto laserSub = nodeh->create_subscription<sensor_msgs::msg::LaserScan>("/scan",1000,&lasercallback); //set up subscription for laser
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
    rclcpp::spin_some(nodeh);
  }

  //create goal pose
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;


  // move to new pose
  navigator.GoToPose(goal_pos);

  //the while loops are so the code waits for the robot to complete its task
  while ( ! navigator.IsTaskComplete() ) {
    
  }
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
