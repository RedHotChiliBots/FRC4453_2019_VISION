
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

constexpr double LOCK_MAX_DY = 25;
constexpr double LOCK_MIN_AREA = 20;

std::vector<std::vector<cv::Point> > getContours(const cv::Mat &frame) {
        cv::Mat frame_hsv;
        cv::cvtColor(frame, frame_hsv, cv::ColorConversionCodes::COLOR_RGB2HSV, -1);

        cv::Mat green(frame_hsv.rows, frame_hsv.cols, CV_8U);
        cv::inRange(frame_hsv, cv::Scalar(70, 100, 120), cv::Scalar(90, 255, 255), green);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(green, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));
        
        return contours;
}

bool isRectGood(const cv::RotatedRect& rect, const std::vector<cv::Point> contour) {
    float cArea = cv::contourArea(contour);
    float rArea = rect.size.area();

    float aArea = (cArea + rArea) / 2.0;

    return std::abs(cArea - rArea) < aArea * 0.25 ;
}

std::vector<cv::RotatedRect> getRects(const std::vector<std::vector<cv::Point> > &contours) {
        std::vector<cv::RotatedRect> rectangles;
        
        for(const auto c : contours) {
            auto r = cv::minAreaRect(c);
            if(isRectGood(r, c)) {
                rectangles.push_back(cv::minAreaRect(c));
            }
        }

        std::sort(rectangles.begin(), rectangles.end(), [](const auto& a, const auto& b) {return a.size.area() > b.size.area();});

        rectangles.resize(2);
        
        return rectangles;
}

bool areRectsGood(const cv::RotatedRect& left, const cv::RotatedRect& right) {
        if(left.size.area() < LOCK_MIN_AREA) {
            return false;
        }
        
        if(right.size.area() < LOCK_MIN_AREA) {
            return false;
        }

        double d_y = right.center.y - left.center.y;

        // if(std::abs(d_y) > LOCK_MAX_DY) {
        //     return false;
        // }

        return true;
}