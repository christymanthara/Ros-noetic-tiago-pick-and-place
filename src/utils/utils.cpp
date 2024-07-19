#include "../headers/utils.h"


/**
 * @brief Convert degrees to radiants
 * @param degrees degrees to be converted
 * @return float value in radiants of degrees
 */
float degreesToRadians(float degrees) {
    return degrees * (M_PI / 180.0);
}

/**
 * @brief Convert polar coordinates to cartesian
 * @param r radius
 * @param theta angle
 * @return CartesianCoordinates struct object containing respective cartesian coords
 */
CartesianCoordinates polarToCartesian(float r, float theta){
    float x = r * cos(theta);
    float y = r * sin(theta);
    return {x, y};
}
