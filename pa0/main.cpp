#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>

#define PI 3.1415926f

int main()
{
    Eigen::Vector3f p(2.0f, 1.0f, 1.0f);
    Eigen::Matrix3f rotate, translation;
    float r = 45.0f / 180.0f * PI;
    rotate  <<  std::cos(r), -std::sin(r), 0,
                std::sin(r),  std::cos(r), 0,
                0, 0, 1;
    translation <<  1, 0, 1,
                    0, 1, 2,
                    0, 0, 1;
    std::cout << translation * rotate * p << std::endl;
    return 0;
}