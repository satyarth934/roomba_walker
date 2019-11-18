/**
 * @copyright  Copyright (c) Satyarth Praveen
 * @copyright  3-Clause BSD License
 * 
 * @file       TurtleBotClass.cpp
 *
 * @brief      This file implements the TurtleBotClass.
 *
 * @author     Satyarth Praveen
 * @date       2019
 */

#include <TurtleBotClass.h>

TurtleBotClass::TurtleBotClass() {
  ros::Rate loop_rate(30);

  laser_subscriber = node_handle.subscribe<sensor_msgs::LaserScan>(
    "/scan", 2, &TurtleBotClass::perceiveScene, this);

  nav_publisher = node_handle.advertise<geometry_msgs::Twist>(
    "/cmd_vel_mux/input/navi", 2);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

TurtleBotClass::~TurtleBotClass() {}

void TurtleBotClass::rotate(int direction) {
  out_msg.linear.x = 0.0;
  out_msg.linear.y = 0.0;
  out_msg.linear.z = 0.0;
  out_msg.angular.x = 0.0;
  out_msg.angular.y = 0.0;
  if (direction == LEFT) {
    out_msg.angular.z = 0.5;
  } else if (direction == RIGHT) {
    out_msg.angular.z = -0.5;
  } else {
    // Do NOTHING and print a warning sign that nothing is happening.
    out_msg.angular.z = 0.0;
  }
  nav_publisher.publish(out_msg);
}

void TurtleBotClass::moveForward() {
  // Move forward.
  out_msg.linear.x = 0.3;
  out_msg.linear.y = 0.0;
  out_msg.linear.z = 0.0;
  out_msg.angular.x = 0.0;
  out_msg.angular.y = 0.0;
  out_msg.angular.z = 0.0;

  nav_publisher.publish(out_msg);
}

void TurtleBotClass::perceiveScene(const sensor_msgs::LaserScanConstPtr &msg) {
  float min_dist = 999999.99;
  for (float dist : msg->ranges) {
    if (std::isnan(dist)) {
      continue;
    }
    min_dist = min_dist > dist ? dist : min_dist;
  }

  if (min_dist < CLOSEST_DISTANCE) {
    float left_avg = 0.0;
    for (int i = 0; i < (msg->ranges.size()/2); ++i) {
      if (std::isnan(msg->ranges[i])) {
        left_avg += CLOSEST_DISTANCE;
      }
      left_avg += msg->ranges[i];
    }
    left_avg = left_avg / (msg->ranges.size()/2);

    float right_avg = 0.0;
    for (int i = (1 + msg->ranges.size()/2); i < msg->ranges.size(); ++i) {
      if (std::isnan(msg->ranges[i])) {
        right_avg += CLOSEST_DISTANCE;
      }
      right_avg += msg->ranges[i];
    }
    right_avg = right_avg / (msg->ranges.size()/2);

    if (left_avg > right_avg) {
      rotate(LEFT);
    } else {
      rotate(RIGHT);
    }
  } else {
    moveForward();
  }
}
