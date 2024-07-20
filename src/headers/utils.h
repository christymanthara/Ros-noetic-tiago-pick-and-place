#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <cmath>
#include "rosnavigatePnP/Detection.h"

enum class Status {
    PickStarted = 0,
    PlaceStarted = 1,
    HandOpen = 2,
    HandClosed = 3,
    ArmHigh = 4,
    ArmLow = 5,
    ActionEnded = 6
};

struct CartesianCoordinates {
    float x;
    float y;
};

struct PoseID {
    CartesianCoordinates pose;
    int id;
};

float degreesToRadians(float degrees);
CartesianCoordinates polarToCartesian(float r, float theta);  // This line was incorrect

#endif // UTILS_H
