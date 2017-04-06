/*******************************************************************************
*   Copyright 2013-2014 EPFL                                                   *
*   Copyright 2013-2014 Quentin Bonnard                                        *
*                                                                              *
*   This file is part of chilitags.                                            *
*                                                                              *
*   Chilitags is free software: you can redistribute it and/or modify          *
*   it under the terms of the Lesser GNU General Public License as             *
*   published by the Free Software Foundation, either version 3 of the         *
*   License, or (at your option) any later version.                            *
*                                                                              *
*   Chilitags is distributed in the hope that it will be useful,               *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*   GNU Lesser General Public License for more details.                        *
*                                                                              *
*   You should have received a copy of the GNU Lesser General Public License   *
*   along with Chilitags.  If not, see <http://www.gnu.org/licenses/>.         *
*******************************************************************************/

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>

#include <chilitags/chilitags.hpp>

#include <opencv2/core/core.hpp> // for cv::Mat
#include <opencv2/core/core_c.h> // CV_AA
#include <opencv2/highgui/highgui.hpp> // for camera capture
#include <opencv2/imgproc/imgproc.hpp> // for camera capture

#include "lookup_table.hpp"
#include "uart.hpp"

using namespace std;
using namespace cv;

const float PI = 3.1415926535;
static const float DEFAULT_SIZE = 20.f;
static const float MARKER_WIDTH = 5.5f;
static const float CM_TO_PIXEL = 4.92;
static const float PIXEL_TO_CM = 1 / CM_TO_PIXEL;
static int camera_id = 0;

/* Note:
- Euclidean space == coordinates in terms of (x, y, z), in that order..
- These functions assume we are using top left corner.
- Angles all in radians, please.
*/

/* Returns the location of the chilitags in euclidean space, where the origin is the camera's location. */
float compute_magnitude(const cv::Vec3f &v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

cv::Vec3f compute_chili_tag_location(const std::string & name, const chilitags::Chilitags3D::TransformMatrix & transformation)
{
    cv::Vec4f reading = {0.f, 0.f, 0.f, 1.f};
    cv::Vec4f raw_homo = transformation * reading;
    cv::Vec3f raw_euclid = cv::Vec3f(
            raw_homo[0] / raw_homo[3],
            raw_homo[1] / raw_homo[3],
            raw_homo[2] / raw_homo[3]);
    cv::Vec3f scaled_raw_euclid = raw_euclid * PIXEL_TO_CM;
    cv::Vec3f tag_location = lookup_chili_tag_location(name);

    return tag_location - cv::Vec3f(scaled_raw_euclid[0], scaled_raw_euclid[2], -scaled_raw_euclid[1]);
}

float align_angle(const float a)
{
    return std::fmod(a, 2 * PI);
}


float compute_chili_tag_angle(const std::string& name, const chilitags::Chilitags3D::TransformMatrix & projection)
{
    cv::Vec4f v1_homo = {0.f, 0.f, 1.f, 1.f};
    cv::Vec4f v2_homo = projection * v1_homo;
    cv::Vec3f v1 = cv::Vec3f(v1_homo[0] / v1_homo[3], v1_homo[1] / v1_homo[3], v1_homo[2] / v1_homo[3]);
    cv::Vec3f v2 = -cv::Vec3f(v2_homo[0] / v2_homo[3], v2_homo[1] / v2_homo[3], v2_homo[2] / v2_homo[3]);
    std::cout << "Tag angle = " << lookup_chili_tag_angle(name) << std::endl;
    return std::acos(v1.dot(v2) / compute_magnitude(v1) / compute_magnitude(v2)) + lookup_chili_tag_angle(name);
}


int pdev;
const char *dev_string = "/dev/ttyUSB0";

struct T {
    int camera_id;
    float x;
    float y;
    float theta;
};

int main(int argc, char* argv[])
{
    pdev = open(dev_string, O_RDWR | O_NOCTTY | O_SYNC);
    uart_set_interface_attribs(pdev, B115200, 0);
    uart_set_blocking(pdev, 0);
    int ptr = 0;


    while (1) {
        T a;
        ssize_t r = read(pdev, ((void*) &a) + ptr, sizeof(T));
        ptr += r;
        if (ptr < sizeof(T)) {
            continue;
        }
        ptr -= sizeof(T);

        std::cout << a.camera_id << ", " << a.x << ", " << a.y << ", " << a.theta << std::endl;
    }

    return 0;
}

