#ifndef VISIONPROC_HPP
#define VISIONPROC_HPP
#include <opencv2/core.hpp>

std::vector<std::vector<cv::Point> > getContours(cv::Mat &frame);
std::vector<cv::RotatedRect> getRects(std::vector<std::vector<cv::Point> > &contours);
bool areRectsGood(cv::RotatedRect& left, cv::RotatedRect& right);

#endif