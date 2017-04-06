bool is_yellow = 1;

/* Returns the location of the chilitags in euclidean space. */
cv::Vec3f lookup_chili_tag_location(const std::string& location)
{
    if (is_yellow) {
        if (location == "tag_110")
            return cv::Vec3f(308.9, 104.0, 49.8);
        if (location == "tag_111")
            return cv::Vec3f(308.9, 104.0, 41.8);
        if (location == "tag_112")
            return cv::Vec3f(302.2, 102.7, 49.8);
        if (location == "tag_113")
            return cv::Vec3f(302.2, 102.7, 41.8);
        if (location == "tag_114")
            return cv::Vec3f(303.5, 96.0, 49.8);
        if (location == "tag_115")
            return cv::Vec3f(303.5, 96.0, 41.8);
        if (location == "tag_116")
            return cv::Vec3f(-2.2, -8.9, 49.8);
        if (location == "tag_117")
            return cv::Vec3f(-2.2, -8.9, 41.8);
        if (location == "tag_118")
            return cv::Vec3f(-3.5, -2.2, 49.8);
        if (location == "tag_119")
            return cv::Vec3f(-3.5, -2.2, 41.8);
        if (location == "tag_120")
            return cv::Vec3f(-8.9, 202.2, 49.8);
        if (location == "tag_121")
            return cv::Vec3f(-8.9, 202.2, 41.8);
        if (location == "tag_122")
            return cv::Vec3f(-2.2, 203.5, 49.8);
        if (location == "tag_123")
            return cv::Vec3f(-2.2, 203.5, 41.8);
    }
    else {
        if (location == "tag_110")
            return cv::Vec3f(-8.9, 96.0, 49.8);
        if (location == "tag_111")
            return cv::Vec3f(-8.9, 96.0, 41.8);
        if (location == "tag_112")
            return cv::Vec3f(-2.2, 97.3, 49.8);
        if (location == "tag_113")
            return cv::Vec3f(-2.2, 97.3, 41.8);
        if (location == "tag_114")
            return cv::Vec3f(-3.5, 104.0, 49.8);
        if (location == "tag_115")
            return cv::Vec3f(-3.5, 104.0, 41.8);
        if (location == "tag_116")
            return cv::Vec3f(308.9, -2.2, 49.8);
        if (location == "tag_117")
            return cv::Vec3f(308.9, -2.2, 41.8);
        if (location == "tag_118")
            return cv::Vec3f(302.2, -3.5, 49.8);
        if (location == "tag_119")
            return cv::Vec3f(302.2, -3.5, 41.8);
        if (location == "tag_120")
            return cv::Vec3f(302.2, 208.9, 49.8);
        if (location == "tag_121")
            return cv::Vec3f(302.2, 208.9, 41.8);
        if (location == "tag_122")
            return cv::Vec3f(303.5, 202.2, 49.8);
        if (location == "tag_123")
            return cv::Vec3f(303.5, 202.2, 41.8);
    }
}

/* Returns the angle of the direction in _radians_  */
float lookup_chili_tag_angle(const std::string& location)
{
    if (is_yellow) {
        if (location == "tag_110")
            return 1.57079632679;
        if (location == "tag_111")
            return 1.57079632679;
        if (location == "tag_112")
            return 3.14159265359;
        if (location == "tag_113")
            return 3.14159265359;
        if (location == "tag_114")
            return 4.71238898038;
        if (location == "tag_115")
            return 4.71238898038;
        if (location == "tag_116")
            return 0.0;
        if (location == "tag_117")
            return 0.0;
        if (location == "tag_118")
            return 1.57079632679;
        if (location == "tag_119")
            return 1.57079632679;
        if (location == "tag_120")
            return 4.71238898038;
        if (location == "tag_121")
            return 4.71238898038;
        if (location == "tag_122")
            return 0.0;
        if (location == "tag_123")
            return 0.0;
    }
    else {
        if (location == "tag_110")
            return 4.71238898038;
        if (location == "tag_111")
            return 4.71238898038;
        if (location == "tag_112")
            return 0.0;
        if (location == "tag_113")
            return 0.0;
        if (location == "tag_114")
            return 1.57079632679;
        if (location == "tag_115")
            return 1.57079632679;
        if (location == "tag_116")
            return 1.57079632679;
        if (location == "tag_117")
            return 1.57079632679;
        if (location == "tag_118")
            return 3.14159265359;
        if (location == "tag_119")
            return 3.14159265359;
        if (location == "tag_120")
            return 3.14159265359;
        if (location == "tag_121")
            return 3.14159265359;
        if (location == "tag_122")
            return 4.71238898038;
        if (location == "tag_123")
            return 4.71238898038;
    }
}
