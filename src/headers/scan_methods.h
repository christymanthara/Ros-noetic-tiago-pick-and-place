#ifndef SCAN_METHODS_H
#define SCAN_METHODS_H

#include <vector>
#include <cmath>
#include <math.h>
#include <utility>
#include <ros/ros.h>

#include "utils.h"

std::vector<std::vector<CartesianCoordinates>> clusterRanges(const std::vector<CartesianCoordinates> &ranges,
															 float th1, float th2);
															 
std::vector<CartesianCoordinates> convertRanges (const std::vector<float> &ranges,
												float angle_min, float angle_increment);
					
std::vector<geometry_msgs::Pose> findCylinders(const std::vector<std::vector<CartesianCoordinates>> &rangeClusters,
				   								float angle_min, float angle_increment);
				   								
#endif // SCAN_METHODS_H
