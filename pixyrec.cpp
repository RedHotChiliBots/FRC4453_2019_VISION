#include <opencv2/opencv.hpp>

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

    pixy->m_link.stop();

    cv::VideoWriter out;
    out.open("pixy.avi");

    for(size_t i = 0; i < 600; i++) {
        uint8_t* bayer = nullptr;
        pixy->m_link.getRawFrame(&bayer);

        cv::Mat m_bayer(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8U, bayer);

        cv::Mat frame;
        cv::cvtColor(m_bayer, frame, cv::ColorConversionCodes::COLOR_BayerBG2BGR, -1);
        out << frame;
    }
}