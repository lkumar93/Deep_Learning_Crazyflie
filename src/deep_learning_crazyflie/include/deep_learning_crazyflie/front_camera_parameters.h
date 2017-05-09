#ifndef FRONT_CAMERA_PARAMETERS_H_
#define FRONT_CAMERA_PARAMETERS_H_

double FocalLength_X_Front = 240.3297;
double FocalLength_Y_Front = 235.144;
double PrincipalPoint_X_Front = 357.36414;
double PrincipalPoint_Y_Front = 269.9525;
double Distortion_Front[5] = {0.1974, -0.2337, -0.00117213, 0.00024763,0.06368892};

float calibration_square_size_front = 2.5 ;//cm
float calibration_pattern_size_front[2] = {9,7}; //rows X cols

#endif //FRONT_CAMERA_PARAMETERS_H_
