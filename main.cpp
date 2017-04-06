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
const char *dev_string = "/dev/ttyS0";

int main(int argc, char* argv[])
{
    camera_id = atoi(argv[1]);
    pdev = open(dev_string, O_RDWR | O_NOCTTY | O_SYNC);
    if (pdev == -1) {
        printf("Failed to open %s!\n", dev_string);
        return -1;
    }
    uart_set_interface_attribs(pdev, B115200, 0);
    uart_set_blocking(pdev, 0);
    cout
        << "Usage: "<< argv[0]
        << " [-c <tag configuration (YAML)>] [-i <camera calibration (YAML)>]\n";

    const char* intrinsicsFilename = 0;
    const char* configFilename = 0;

    for( int i = 1; i < argc; i++ )
    {
        if( strcmp(argv[i], "-c") == 0 )
            configFilename = argv[++i];
        else if( strcmp(argv[i], "-i") == 0 )
            intrinsicsFilename = argv[++i];
    }

    /*****************************/
    /*    Init camera capture    */
    /*****************************/
    int cameraIndex = 0;
    cv::VideoCapture capture(cameraIndex);
    if (!capture.isOpened())
    {
        cerr << "Unable to initialise video capture.\n";
        return 1;
    }

    /******************************/
    /* Setting up pose estimation */
    /******************************/
#ifdef OPENCV3
    float inputWidth  = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    float inputHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
#else
    float inputWidth  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    float inputHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
#endif

    chilitags::Chilitags3D chilitags3D(Size(inputWidth, inputHeight));

    if (configFilename) chilitags3D.readTagConfiguration(configFilename);

    if (intrinsicsFilename) {
        Size calibratedImageSize = chilitags3D.readCalibration(intrinsicsFilename);
#ifdef OPENCV3
        capture.set(cv::CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);
#else
        capture.set(CV_CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);
#endif
    }

    cv::Mat projectionMat = cv::Mat::zeros(4,4,CV_32F);
    chilitags3D.getCameraMatrix().copyTo(projectionMat(cv::Rect(0,0,3,3)));
    cv::Matx44f projection = projectionMat;
    projection(3,2) = 1;

    /*****************************/
    /*             Go!           */
    /*****************************/

    for (; 'q' != (char) cv::waitKey(10); ) {
        cv::Mat inputImage;
        capture.read(inputImage);
        cv::Mat outputImage = inputImage.clone();

        int N = 0;
        cv::Vec3f camera_location;
        float angle = 0;

        for (auto& kv : chilitags3D.estimate(inputImage)) {

            /* the location of the top left corner in euclidean space. */
            camera_location += compute_chili_tag_location(kv.first, kv.second);
            angle += compute_chili_tag_angle(kv.first, kv.second);
            N++;
        }

        
        if (N != 0) {
            angle = angle / N;
            camera_location = camera_location / N;

            float x = camera_location[0];
            float y = camera_location[1];
            float theta = angle;
            uint8_t buffer[4 * sizeof(float)];

            std::cout << "x = " << x << ", y = " << y << ", theta = " << theta << std::endl;
            memcpy(buffer + 0, (void*) &camera_id, sizeof(int));
            memcpy(buffer + 4, (void*) &x, sizeof(float));
            memcpy(buffer + 8, (void*) &y, sizeof(float));
            memcpy(buffer + 12, (void*) &theta, sizeof(float));
            write(pdev, (void*) buffer, 4 * sizeof(float));
        } else {
            std::cout << "No tags detected" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    capture.release();

    return 0;
}

