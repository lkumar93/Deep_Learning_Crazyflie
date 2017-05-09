#ifndef BOTTOM_CAMERA_PARAMETERS_H_
#define BOTTOM_CAMERA_PARAMETERS_H_

double FocalLength_X_Bottom = 668.1563;
double FocalLength_Y_Bottom = 545.1772;
double PrincipalPoint_X_Bottom = 985.7908;
double PrincipalPoint_Y_Bottom = 617.8635;
double Distortion_Bottom[5] = {0.20251939, -0.24466239, -0.00098169, 0.00123158,0.06783239};

float calibration_square_size_bottom = 2.5 ;//cm
float calibration_pattern_size_bottom[2] = {9,7}; //rows X cols

#endif //BOTTOM_CAMERA_PARAMETERS_H_
