/**
 * @copyright  Copyright (c) Satyarth Praveen
 * @copyright  3-Clause BSD License
 * 
 * @file       TurtleBotClass.h
 *
 * @brief      Header file for the TurtleBotClass.cpp file.
 *
 * @author     Satyarth Praveen
 * @date       2019
 */

#ifndef TURTLEBOTCLASS_H
#define TURTLEBOTCLASS_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

/**
 * @brief      This class describes a TurtleBottomClass.
 */
class TurtleBotClass {
private:
  static const int LEFT = 0;
  static const int RIGHT = 1;
  static constexpr float CLOSEST_DISTANCE = 0.5; // distance in meters.

  ros::NodeHandle node_handle;
  ros::Subscriber laser_subscriber;
  ros::Publisher nav_publisher;

public:
  geometry_msgs::Twist out_msg;

  /**
   * @brief      Constructs a new instance.
   */
  TurtleBotClass();
  /**
   * @brief      Destroys the object.
   */
  ~TurtleBotClass();
  
  /**
   * @brief      moves the robot forward.
   * 
   * @return     none
   */
  void moveForward();

  /**
   * @brief      rotates the robot in-place.
   *
   * @param      direction  The direction in which the robot is to be rotated.
   * 
   * @return     none
   */
  void rotate(int direction);

  /**
   * @brief      processes the scene to decide where the to keep moving forward or take a turn.
   *
   * @param      msg   The subscribed message
   * 
   * @return     none
   */
  void perceiveScene(const sensor_msgs::LaserScanConstPtr &msg);
};

#endif