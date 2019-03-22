#include "test.hpp"
#include "visionproc.hpp"

TEST(noise_negative) {

    for(size_t i = 0; i < 1000; i++) {
        cv::Mat frame(400, 400, CV_8UC3);
        cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        auto contours = getContours(frame);
        auto rects = getRects(contours);
        if(rects.size() < 2) {
            continue;
        }
        auto left = rects[0];
        auto right = rects[1];
        TEST_ASSERT(!areRectsGood(left, right), "Found rects in noise!");
    }

    TEST_PASS
}

START_TESTSUITE_REGISTRY(visionproc)
REGISTER_TEST(noise_negative)
END_TESTSUITE_REGISTRY