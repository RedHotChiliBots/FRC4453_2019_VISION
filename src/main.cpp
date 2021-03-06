#define _USE_MATH_DEFINES // Needed on MSVC for M_PI.
#include <cmath>
#include <algorithm>
#include <future>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#ifdef DEV_TEST
#include <opencv2/highgui.hpp>
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" // Makes dumb warnings go away.
#endif
#include <libpixyusb2.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gcem.hpp>
#include <networktables/NetworkTableInstance.h>

// #include "cammath.hpp"
// #include "transform.hpp"
// #include "conversion.hpp"

#include "connectionwaiter.hpp"
#include "pixyfinder.hpp"
#include "visionproc.hpp"

#if (defined(_WIN32) | defined(DEV_TEST))
#define LOG_FILE "./log/Vision.txt"
#else
#define LOG_FILE "/var/log/Vision.txt"
#endif

// Converts radians to degrees.
constexpr double rad2deg(double v) {
    return v * (180.0 / M_PI);
}

// Converts degrees to radians.
constexpr double deg2rad(double v) {
    return v / (180.0 / M_PI);
}

constexpr double CAM_HEIGHT = 14.5; // Height of the camera in inches.
constexpr double CAM_DOWNPITCH = deg2rad(-40.0); // Angle of camera in radians.

constexpr double CAM_REAR_OFFSET = 5; // X offset of rear camera in inches. 
constexpr double CAM_FRONT_OFFSET = 5;

constexpr double CAM_FOV_Y = deg2rad(60.0);
constexpr double CAM_RES_X = 208;
constexpr double CAM_RES_Y = 316;
constexpr double CAM_ASPECT_RATIO = CAM_RES_X / CAM_RES_Y;
constexpr double CAM_FOV_X = CAM_FOV_Y * CAM_ASPECT_RATIO;

constexpr double CAM_FX_PIXEL = (CAM_RES_X/2.0) / gcem::tan(CAM_FOV_X / 2.0);
constexpr double CAM_FY_PIXEL = (CAM_RES_Y/2.0) / gcem::tan(CAM_FOV_Y / 2.0);

constexpr double CAM_CENTERX = CAM_RES_X / 2.0;
constexpr double CAM_CENTERY = CAM_RES_Y / 2.0;

cv::Mat getCamMat() {
    // Fx,  0, Cx
    //  0, Fy, Cy
    //  0,  0,  1

    return (cv::Mat_<double>(3,3) << CAM_FX_PIXEL, 0, CAM_CENTERX, 0, CAM_FY_PIXEL, CAM_CENTERY, 0, 0, 1);
}

constexpr uint32_t PIXY_FRONT_ID = 0xE4E35363; // UID of front camera.
constexpr uint32_t PIXY_REAR_ID = 0xF1435B59; // UID of rear camera.

// Calculation of strip corner coordinates

// Basic info from game manual.
constexpr double STRIP_ANGLE_ABS = deg2rad(14.5);
constexpr double STRIP_ANGLE_TOL = 1.5;
constexpr double STRIP_HEIGHT = 5.5;
constexpr double STRIP_WIDTH = 2;
constexpr double STRIP_OFFSET = 8/2;

// Coords of unrotated strip, with center as (0,0).
constexpr double STRIP_TOPLEFT_X = (-STRIP_WIDTH/2.0);
constexpr double STRIP_TOPLEFT_Y = (STRIP_HEIGHT/2.0);
constexpr double STRIP_TOPRIGHT_X = (STRIP_WIDTH/2.0);
constexpr double STRIP_TOPRIGHT_Y = (STRIP_HEIGHT/2.0);
constexpr double STRIP_BOTLEFT_X = (-STRIP_WIDTH/2.0);
constexpr double STRIP_BOTLEFT_Y = (-STRIP_HEIGHT/2.0);
constexpr double STRIP_BOTRIGHT_X = (STRIP_WIDTH/2.0);
constexpr double STRIP_BOTRIGHT_Y = (-STRIP_HEIGHT/2.0);

// Helper function for rotation.
constexpr double rotX(const double x, const double y, const double angle) {
    return x * gcem::cos(angle) - y * gcem::sin(angle);
}

// Helper function for rotation.
constexpr double rotY(const double x, const double y, const double angle) {
    return y * gcem::cos(angle) + x * gcem::sin(angle);
}

// Left strip coordinates with center as (0,0).
constexpr double LSTRIP_TOPLEFT_X_LOCAL  = rotX(STRIP_TOPLEFT_X,  STRIP_TOPLEFT_Y,  -STRIP_ANGLE_ABS);
constexpr double LSTRIP_TOPLEFT_Y_LOCAL  = rotY(STRIP_TOPLEFT_X,  STRIP_TOPLEFT_Y,  -STRIP_ANGLE_ABS);
constexpr double LSTRIP_TOPRIGHT_X_LOCAL = rotX(STRIP_TOPRIGHT_X, STRIP_TOPRIGHT_Y, -STRIP_ANGLE_ABS);
constexpr double LSTRIP_TOPRIGHT_Y_LOCAL = rotY(STRIP_TOPRIGHT_X, STRIP_TOPRIGHT_Y, -STRIP_ANGLE_ABS);
constexpr double LSTRIP_BOTLEFT_X_LOCAL  = rotX(STRIP_BOTLEFT_X,  STRIP_BOTLEFT_Y,  -STRIP_ANGLE_ABS);
constexpr double LSTRIP_BOTLEFT_Y_LOCAL  = rotY(STRIP_BOTLEFT_X,  STRIP_BOTLEFT_Y,  -STRIP_ANGLE_ABS);
constexpr double LSTRIP_BOTRIGHT_X_LOCAL = rotX(STRIP_BOTRIGHT_X, STRIP_BOTRIGHT_Y, -STRIP_ANGLE_ABS);
constexpr double LSTRIP_BOTRIGHT_Y_LOCAL = rotY(STRIP_BOTRIGHT_X, STRIP_BOTRIGHT_Y, -STRIP_ANGLE_ABS);

// Right strip coordinates with center as (0,0).
constexpr double RSTRIP_TOPLEFT_X_LOCAL  = rotX(STRIP_TOPLEFT_X,  STRIP_TOPLEFT_Y,   STRIP_ANGLE_ABS);
constexpr double RSTRIP_TOPLEFT_Y_LOCAL  = rotY(STRIP_TOPLEFT_X,  STRIP_TOPLEFT_Y,   STRIP_ANGLE_ABS);
constexpr double RSTRIP_TOPRIGHT_X_LOCAL = rotX(STRIP_TOPRIGHT_X, STRIP_TOPRIGHT_Y,  STRIP_ANGLE_ABS);
constexpr double RSTRIP_TOPRIGHT_Y_LOCAL = rotY(STRIP_TOPRIGHT_X, STRIP_TOPRIGHT_Y,  STRIP_ANGLE_ABS);
constexpr double RSTRIP_BOTLEFT_X_LOCAL  = rotX(STRIP_BOTLEFT_X,  STRIP_BOTLEFT_Y,   STRIP_ANGLE_ABS);
constexpr double RSTRIP_BOTLEFT_Y_LOCAL  = rotY(STRIP_BOTLEFT_X,  STRIP_BOTLEFT_Y,   STRIP_ANGLE_ABS);
constexpr double RSTRIP_BOTRIGHT_X_LOCAL = rotX(STRIP_BOTRIGHT_X, STRIP_BOTRIGHT_Y,  STRIP_ANGLE_ABS);
constexpr double RSTRIP_BOTRIGHT_Y_LOCAL = rotY(STRIP_BOTRIGHT_X, STRIP_BOTRIGHT_Y,  STRIP_ANGLE_ABS);

// Left strip coordinates.
constexpr double LSTRIP_TOPLEFT_X  = LSTRIP_TOPLEFT_X_LOCAL  - (STRIP_OFFSET + LSTRIP_TOPRIGHT_X_LOCAL);
constexpr double LSTRIP_TOPLEFT_Y  = LSTRIP_TOPLEFT_Y_LOCAL;
constexpr double LSTRIP_TOPRIGHT_X = LSTRIP_TOPRIGHT_X_LOCAL - (STRIP_OFFSET + LSTRIP_TOPRIGHT_X_LOCAL);
constexpr double LSTRIP_TOPRIGHT_Y = LSTRIP_TOPRIGHT_Y_LOCAL;
constexpr double LSTRIP_BOTLEFT_X  = LSTRIP_BOTLEFT_X_LOCAL  - (STRIP_OFFSET + LSTRIP_TOPRIGHT_X_LOCAL);
constexpr double LSTRIP_BOTLEFT_Y  = LSTRIP_BOTLEFT_Y_LOCAL;
constexpr double LSTRIP_BOTRIGHT_X = LSTRIP_BOTRIGHT_X_LOCAL - (STRIP_OFFSET + LSTRIP_TOPRIGHT_X_LOCAL);
constexpr double LSTRIP_BOTRIGHT_Y = LSTRIP_BOTRIGHT_Y_LOCAL;

// Right strip coordinates.
constexpr double RSTRIP_TOPLEFT_X  = RSTRIP_TOPLEFT_X_LOCAL  + (STRIP_OFFSET - RSTRIP_TOPLEFT_X_LOCAL);
constexpr double RSTRIP_TOPLEFT_Y  = RSTRIP_TOPLEFT_Y_LOCAL;
constexpr double RSTRIP_TOPRIGHT_X = RSTRIP_TOPRIGHT_X_LOCAL + (STRIP_OFFSET - RSTRIP_TOPLEFT_X_LOCAL);
constexpr double RSTRIP_TOPRIGHT_Y = RSTRIP_TOPRIGHT_Y_LOCAL;
constexpr double RSTRIP_BOTLEFT_X  = RSTRIP_BOTLEFT_X_LOCAL  + (STRIP_OFFSET - RSTRIP_TOPLEFT_X_LOCAL);
constexpr double RSTRIP_BOTLEFT_Y  = RSTRIP_BOTLEFT_Y_LOCAL;
constexpr double RSTRIP_BOTRIGHT_X = RSTRIP_BOTRIGHT_X_LOCAL + (STRIP_OFFSET - RSTRIP_TOPLEFT_X_LOCAL);
constexpr double RSTRIP_BOTRIGHT_Y = RSTRIP_BOTRIGHT_Y_LOCAL;

std::vector<cv::Point3f> getStrips() {
    std::vector<cv::Point3f> ret;
    ret.push_back(cv::Point3f(LSTRIP_TOPLEFT_X,  LSTRIP_TOPLEFT_Y,  0));
    ret.push_back(cv::Point3f(LSTRIP_TOPRIGHT_X, LSTRIP_TOPRIGHT_Y, 0));
    ret.push_back(cv::Point3f(LSTRIP_BOTLEFT_X,  LSTRIP_BOTLEFT_Y,  0));
    ret.push_back(cv::Point3f(LSTRIP_BOTRIGHT_X, LSTRIP_BOTRIGHT_Y, 0));
    ret.push_back(cv::Point3f(RSTRIP_TOPLEFT_X,  RSTRIP_TOPLEFT_Y,  0));
    ret.push_back(cv::Point3f(RSTRIP_TOPRIGHT_X, RSTRIP_TOPRIGHT_Y, 0));
    ret.push_back(cv::Point3f(RSTRIP_BOTLEFT_X,  RSTRIP_BOTLEFT_Y,  0));
    ret.push_back(cv::Point3f(RSTRIP_BOTRIGHT_X, RSTRIP_BOTRIGHT_Y, 0));
    return ret;
}

cv::Point2f lineLineIntersection(const cv::Point2f& A, const cv::Point2f& B, const cv::Point2f& C, const cv::Point2f& D) 
{ 
    // Line AB represented as a1x + b1y = c1 
    float a1 = B.y - A.y; 
    float b1 = A.x - B.x; 
    float c1 = a1*(A.x) + b1*(A.y); 
  
    // Line CD represented as a2x + b2y = c2 
    float a2 = D.y - C.y; 
    float b2 = C.x - D.x; 
    float c2 = a2*(C.x)+ b2*(C.x); 
    float determinant = a1*b2 - a2*b1; 
  
    if (determinant == 0) 
    { 
        // The lines are parallel. This is simplified 
        // by returning a pair of FLT_MAX 
        return cv::Point2f(FLT_MAX, FLT_MAX); 
    } 
    else
    { 
        float x = (b2*c1 - b1*c2)/determinant; 
        float y = (a1*c2 - a2*c1)/determinant; 
        return cv::Point2f(x, y); 
    } 
} 

std::vector<cv::Point2f> pointsFromRects(const cv::RotatedRect& left, const cv::RotatedRect& right
    #ifdef DEV_TEST
    , const cv::Mat& frame 
    #endif
) {
    std::vector<cv::Point2f> ret;

    cv::Point2f left_rect_pts[4];
    left.points(left_rect_pts);
    
    cv::Point2f right_rect_pts[4];
    right.points(right_rect_pts);

    const auto center = (left.center + right.center) / 2.0;

    const auto vert_int = lineLineIntersection(left_rect_pts[0], left_rect_pts[1], right_rect_pts[0], right_rect_pts[1]);
    const auto horz_int = lineLineIntersection(left_rect_pts[1], left_rect_pts[2], right_rect_pts[1], right_rect_pts[2]);

    const auto vert_dist = cv::norm(vert_int - center);
    const auto horz_dist = cv::norm(horz_int - center);

    #ifdef DEV_TEST
    cv::Mat draw_frame(frame.clone());
    cv::line(draw_frame, center, vert_int, cv::Scalar(255, 255, 0));
    cv::line(draw_frame, center, horz_int, cv::Scalar(255, 0, 255));

    cv::line(draw_frame, left_rect_pts[0], left_rect_pts[1], cv::Scalar(0, 0, 255));
    cv::line(draw_frame, left_rect_pts[1], left_rect_pts[2], cv::Scalar(0, 0, 255));
    cv::line(draw_frame, left_rect_pts[2], left_rect_pts[3], cv::Scalar(0, 0, 255));

    cv::line(draw_frame, right_rect_pts[0], right_rect_pts[1], cv::Scalar(255, 0, 0));
    cv::line(draw_frame, right_rect_pts[1], right_rect_pts[2], cv::Scalar(255, 0, 0));
    cv::line(draw_frame, right_rect_pts[2], right_rect_pts[3], cv::Scalar(255, 0, 0));
    cv::cvtColor(draw_frame, draw_frame, cv::ColorConversionCodes::COLOR_RGB2BGR, -1);

    cv::imshow("Rects", draw_frame);
    cv::waitKey(1);
    #endif

    if(vert_dist > horz_dist) {
        // Vertical
        if((vert_int - center).y > 0) {
            // Upright
            ret.push_back( left_rect_pts[1]);
            ret.push_back( left_rect_pts[2]);
            ret.push_back( left_rect_pts[0]);
            ret.push_back( left_rect_pts[3]);
            ret.push_back(right_rect_pts[1]);
            ret.push_back(right_rect_pts[2]);
            ret.push_back(right_rect_pts[0]);
            ret.push_back(right_rect_pts[3]);
        } else {
            // Upside-down
            ret.push_back( left_rect_pts[3]);
            ret.push_back( left_rect_pts[0]);
            ret.push_back( left_rect_pts[2]);
            ret.push_back( left_rect_pts[1]);
            ret.push_back(right_rect_pts[3]);
            ret.push_back(right_rect_pts[0]);
            ret.push_back(right_rect_pts[2]);
            ret.push_back(right_rect_pts[1]);
        }
    } else {
        // TODO
        spdlog::error("aaaaaaa {}, {}", vert_dist, horz_dist);
    }

    return ret;
}

// Processes vectors from a pixy and puts results in NetworkTables.
void thread_fn(std::shared_ptr<PixyFinder> p, std::shared_ptr<nt::NetworkTable> table, uint32_t id) {
    spdlog::debug("Camera thread started for id {0:x}", id);
    while(true) { // Do this forever.
        std::shared_ptr<Pixy2> pixy;
        
        try {
            pixy = p->get(id); // Retrieve pixy.
        } catch (...) {
            table->PutBoolean("Lock", false);
            table->PutBoolean("Ok", false);
            table->GetInstance().Flush();
            p->update(id);
            continue; // Skip loop.
        }

        pixy->m_link.stop();

        int _res;

        pixy->m_link.callChirp("cam_setAEC", INT8(1), END_OUT_ARGS, &_res, END_IN_ARGS);
        //pixy->m_link.callChirp("cam_setAWB", INT8(1), END_OUT_ARGS, &_res, END_IN_ARGS);


        uint8_t* bayer = nullptr;
        pixy->m_link.getRawFrame(&bayer);

        if(bayer == nullptr) {
            pixy->setLED(255, 0, 0);
            table->PutBoolean("Lock", false);
            table->PutBoolean("Ok", false);
            continue;
        }

        cv::Mat m_bayer(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8U, bayer);

        cv::Mat frame, frame_hsv;
        cv::cvtColor(m_bayer, frame, cv::ColorConversionCodes::COLOR_BayerRG2RGB, -1);
        cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);


        auto contours = getContours(frame);

        auto rectangles = getRects(contours);
        
        table->PutNumber("NumObjects", rectangles.size());

        
        #ifdef DEV_TEST
        cv::Mat display_frame = frame.clone();
        cv::drawContours(display_frame, contours, -1, cv::Scalar(0, 0, 255));

        for(size_t i = 0; i < rectangles.size(); i++) {
            auto bb = rectangles[i].boundingRect();
            cv::rectangle(display_frame, bb, cv::Scalar(255, 0, 0));
            cv::putText(display_frame, std::to_string(i), bb.tl(), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0));
        }

        #endif



        if(rectangles.size() < 2) {
            pixy->setLED(255, 0, 0);
            table->PutBoolean("Lock", false);
            table->PutBoolean("Ok", true);
            #ifdef DEV_TEST
            cv::cvtColor(display_frame, display_frame, cv::ColorConversionCodes::COLOR_RGB2BGR, -1);
            cv::imshow("Frame", display_frame);
            cv::waitKey(1);
            #endif
            
            continue;
        }

        cv::RotatedRect left = rectangles[0];
        cv::RotatedRect right = rectangles[1];

        if(!areRectsGood(left, right)) {
            pixy->setLED(0, 0, 255);
            table->PutBoolean("Lock", false);
            table->PutBoolean("Ok", true);

            #ifdef DEV_TEST
            cv::cvtColor(display_frame, display_frame, cv::ColorConversionCodes::COLOR_RGB2BGR, -1);
            cv::imshow("Frame", display_frame);
            cv::waitKey(1);
            #endif
            

            continue;
        }

        if(left.center.x > right.center.x) {
            std::swap(left, right);
        }

        cv::Mat camMat = getCamMat();

        auto object_pts = getStrips();
        auto image_pts = pointsFromRects(left, right
                                        #ifdef DEV_TEST
                                            ,frame
                                        #endif
                                        );

        if(image_pts.size() != 8) {
            continue;
        }

        cv::Mat r_vec(1, 3, CV_64F);
        cv::Mat T_cv(1, 3, CV_64F);

        auto res = cv::solvePnPRansac(object_pts, image_pts, camMat, std::vector<double>(), r_vec, T_cv, false, 100, 8.0f, 0.95);
        
        if(!res) {
            pixy->setLED(0, 0, 255);
            table->PutBoolean("Lock", false);
            table->PutBoolean("Ok", true);

            #ifdef DEV_TEST
            cv::cvtColor(display_frame, display_frame, cv::ColorConversionCodes::COLOR_RGB2BGR, -1);
            cv::imshow("Frame", display_frame);
            cv::waitKey(1);
            #endif

            continue;
        }

        #ifdef DEV_TEST
        std::vector<cv::Point2f> draw_imgpts; 
        auto draw_objpts = getStrips();
        cv::projectPoints(draw_objpts, r_vec, T_cv, camMat, std::vector<double>(), draw_imgpts);

        for(const auto& p : draw_imgpts) {
            cv::circle(display_frame, p, 3, cv::Scalar(0, 255, 0));
        }

        cv::cvtColor(display_frame, display_frame, cv::ColorConversionCodes::COLOR_RGB2BGR, -1);
        cv::imshow("Frame", display_frame);
        cv::waitKey(1);
        #endif

        cv::Mat r_mat;
        cv::Rodrigues(r_vec, r_mat);

        Eigen::Matrix<double, 3, 3> r_m((double*) r_mat.ptr());

        Eigen::Quaterniond r(r_m);

        Eigen::Vector<double, 3> T_vec((double*) T_cv.ptr());
        Eigen::Translation3d T(T_vec);

        double servo_rot = deg2rad(table->GetNumber("ServoRot", 0));
        Eigen::Quaterniond servo(Eigen::AngleAxisd(servo_rot, Eigen::Vector3d::UnitY()));

        Eigen::Translation3d offset(id == PIXY_FRONT_ID ? CAM_FRONT_OFFSET : CAM_REAR_OFFSET, 0, 0);

        auto transform_cam = r * T;
        auto transform = transform_cam * servo.inverse() * offset.inverse();

        auto pos_cam = transform_cam * Eigen::Vector<double, 3>(0, 0, 0);
        auto pos = transform * Eigen::Vector<double, 3>(0, 0, 0);

        double servoError = std::atan(pos_cam.x() / pos_cam.z());

        double turn = std::atan(pos.x() / pos.z());
        double strafe = pos.x();

        table->PutNumber("Turn", rad2deg(turn));
        table->PutNumber("Strafe", rad2deg(strafe));
        table->PutNumber("ServoError", rad2deg(servoError));

        pixy->setLED(0, 255, 0);

        table->PutBoolean("Lock", true);
        table->PutBoolean("Ok", true);
    }
}

int main() {
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        //std::filesystem::create_directory("log");

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(LOG_FILE, 1048576 * 5, 3);

        auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("Vision", {console_sink, file_sink}));
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
    }

    spdlog::info("Vision Starting...");

    spdlog::debug("Target Points: ");
    for(auto p : getStrips()) {
        spdlog::debug("({}, {})", p.x, p.y, p.z);
    }

    // In another thread, setup NetworkTables.
    auto networktables_f = std::async([]() {
        spdlog::debug("NT Setup: Connecting to NetworkTables...");
        auto nt_inst = nt::NetworkTableInstance::GetDefault();
        nt_inst.SetNetworkIdentity("Vision");
        #if DEV_TEST
        nt_inst.StartServer(); // Start a server for testing.
        #else
        ConnectionWaiter waiter; // Create our waiter.
        nt_inst.AddConnectionListener(waiter.listener, true); // Register the listener.
        nt_inst.StartClientTeam(4453); // Start client.
        waiter.wait_for_connection(); // Wait for a connection.
        #endif
        spdlog::debug("NT Setup: Connected to NetworkTables.");
        return nt_inst;
    });

    std::shared_ptr<PixyFinder> p(new PixyFinder()); // Create PixyFinder.

    spdlog::debug("Starting PixyFinder thread...");

    // Thread to run updates.
    std::thread thread_pixyfinder([p]() {
        p->do_updates();
    });

    spdlog::debug("Started PixyFinder thread.");

    p->update(PIXY_FRONT_ID);
    p->update(PIXY_REAR_ID);


    auto networktables = networktables_f.get(); // Wait for NT setup, get result.

    auto table = networktables.GetTable("Vision");

    auto front_table = table->GetSubTable("Front");
    auto rear_table = table->GetSubTable("Rear");

    // Thread for front camera.
    std::thread thread_front(thread_fn, p, front_table, PIXY_FRONT_ID);
    // Thread for rear camera.
    std::thread thread_rear(thread_fn, p, rear_table, PIXY_REAR_ID);

    spdlog::info("Vision Running!");

    // Wait for threads (which should never exit).
    thread_front.join();
    thread_rear.join();
    thread_pixyfinder.join();
}
