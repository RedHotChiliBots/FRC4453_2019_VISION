#include <opencv2/opencv.hpp>
#include <iostream>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" // Makes dumb warnings go away.
#endif
#include <libpixyusb2.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif


int main() {
    auto pixy = new Pixy2();
    if(pixy->init() < 0) {
        std::cout << "Cannot open pixy!" << std::endl;
        return -1;
    }

    pixy->m_link.stop();

    cv::VideoWriter out;
    out.open("pixy.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(316, 208), true);
    if(!out.isOpened()) {
        std::cout << "Cannot open file!" << std::endl;
        return -2;
    }

    for(size_t i = 0; i < 1200; i++) {
        uint8_t* bayer = nullptr;
        pixy->m_link.getRawFrame(&bayer);

        cv::Mat m_bayer(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8U, bayer);

        cv::Mat frame;
        cv::cvtColor(m_bayer, frame, cv::ColorConversionCodes::COLOR_BayerRG2BGR, 3);

        if(i == 0) {
            std::cout << "Real Size: " << frame.cols << ", " << frame.rows << std::endl;
        }

        out << frame;
        std::cout << i << std::endl;
    }
}