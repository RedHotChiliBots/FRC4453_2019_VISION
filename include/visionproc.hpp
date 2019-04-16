#ifndef VISIONPROC_HPP
#define VISIONPROC_HPP
#include <opencv2/core.hpp>

std::vector<std::vector<cv::Point> > getContours(const cv::Mat &frame);
std::vector<cv::RotatedRect> getRects(const std::vector<std::vector<cv::Point> > &contours);
bool areRectsGood(const cv::RotatedRect& left, const cv::RotatedRect& right);

#endif