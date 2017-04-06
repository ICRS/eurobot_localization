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

#include <chilitags/chilitags.hpp>

#include <opencv2/core/core.hpp> // for cv::Mat
#include <opencv2/core/core_c.h> // CV_AA
#include <opencv2/highgui/highgui.hpp> // for camera capture
#include <opencv2/imgproc/imgproc.hpp> // for camera capture

#include "lookup_table.hpp"

using namespace std;
using namespace cv;

const float PI = 3.1415926535;

/* Note:
- Euclidean space == coordinates in terms of (x, y, z), in that order..
- These functions assume we are using top left corner.
- Angles all in radians, please.
*/

/* Returns the location of the chilitags in euclidean space, where the origin is the camera's location. */
cv::Vec3f compute_chili_tag_location(const std::string & name, const chilitags::Chilitags3D::TransformMatrix & transformation)
{
    cv::Vec4f reading = {0.f, 0.f, 0.f, 1.f};
    cv::Vec4f raw_homo = transformation * reading;
    cv::Vec3f raw_euclid = cv::Vec3f(
            raw_homo[0] / raw_homo[3],
            raw_homo[1] / raw_homo[3],
            raw_homo[2] / raw_homo[3]);
    cv::Vec3f tag_location = lookup_chili_tag_location(name);

    std::cout << "Name = " << name << std::endl;
    std::cout << "euclid = ("
            << raw_euclid[0] << ", "
            << raw_euclid[1] << ", "
            <<  raw_euclid[2] << ")" << std::endl;
    std::cout << "tag location = ("
            << tag_location[0] << ", "
            << tag_location[1] << ", "
            <<  tag_location[2] << ")" << std::endl;
    return cv::Vec3f(raw_euclid[1], raw_euclid[2], -raw_euclid[0]) - lookup_chili_tag_location(name);
}

float compute_magnitude(const cv::Vec3f &v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float align_angle(const float a)
{
    return std::fmod(a, 2 * PI);
}


float compute_chili_tag_angle(const std::string& name, const chilitags::Chilitags3D::TransformMatrix & projection)
{
    cv::Vec4f v1_homo = {0.f, 0.f, 0.f, 1.f};
    cv::Vec4f v2_homo = projection * v1_homo;
    cv::Vec3f v1 = cv::Vec3f(v1_homo[0] / v1_homo[3], v1_homo[1] / v1_homo[3], v1_homo[2] / v1_homo[3]);
    cv::Vec3f v2 = cv::Vec3f(v2_homo[0] / v2_homo[3], v2_homo[1] / v2_homo[3], v2_homo[2] / v2_homo[3]);
    return align_angle(std::acos(v1.dot(v2) / compute_magnitude(v1) / compute_magnitude(v2)) + lookup_chili_tag_angle(name));
}

int main(int argc, char* argv[])
{
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
    cv::namedWindow("Pose Estimation");

    for (; 'q' != (char) cv::waitKey(10); ) {
        cv::Mat inputImage;
        capture.read(inputImage);
        cv::Mat outputImage = inputImage.clone();

        for (auto& kv : chilitags3D.estimate(inputImage)) {

            /* the location of the top left corner in euclidean space. */
            cv::Vec3f camera_location = compute_chili_tag_location(kv.first, kv.second);
            float angle = compute_chili_tag_angle(kv.first, kv.second);

            std::cout << "angle = " << (angle * 180.0 / PI) << std::endl;
            std::cout << "location = ("
                    << camera_location[0] << ", "
                    << camera_location[1] << ", "
                    <<  camera_location[2] << ")" << std::endl;

            // the original program goes here.

            static const float DEFAULT_SIZE = 20.f;
            static const cv::Vec4f UNITS[4] {
                {0.f, 0.f, 0.f, 1.f},
                {DEFAULT_SIZE, 0.f, 0.f, 1.f},
                {0.f, DEFAULT_SIZE, 0.f, 1.f},
                {0.f, 0.f, DEFAULT_SIZE, 1.f},
            };

            cv::Matx44f transformation = kv.second;
            cv::Vec4f referential[4] = {
                projection*transformation*UNITS[0],
                projection*transformation*UNITS[1],
                projection*transformation*UNITS[2],
                projection*transformation*UNITS[3],
            };

            std::vector<cv::Point2f> t2DPoints;
            for (auto homogenousPoint : referential)
                t2DPoints.push_back(cv::Point2f(
                                        homogenousPoint[0]/homogenousPoint[3],
                                        homogenousPoint[1]/homogenousPoint[3]));

            static const int SHIFT = 16;
            static const float PRECISION = 1<<SHIFT;
            static const std::string AXIS_NAMES[3] = { "x", "y", "z" };
            static const cv::Scalar AXIS_COLORS[3] = {
                {0,0,255},{0,255,0},{255,0,0},
            };
            for (int i : {1,2,3}) {
                cv::line(
                    outputImage,
                    PRECISION*t2DPoints[0],
                    PRECISION*t2DPoints[i],
                    AXIS_COLORS[i-1],
#ifdef OPENCV3
                    1, cv::LINE_AA, SHIFT);
#else
                    1, CV_AA, SHIFT);
#endif
                cv::putText(outputImage, AXIS_NAMES[i-1], t2DPoints[i],
                            cv::FONT_HERSHEY_SIMPLEX, 0.5f, AXIS_COLORS[i-1]);
            }

            cv::putText(outputImage, kv.first, t2DPoints[0],
                        cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(255,255,255));
        }

        cv::imshow("Pose Estimation", outputImage);
    }

    cv::destroyWindow("Pose Estimation");
    capture.release();

    return 0;
}

