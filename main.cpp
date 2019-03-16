#define _USE_MATH_DEFINES // Needed on MSVC for M_PI.
#include <cmath>
#include <algorithm>
#include <future>
#include <sstream>
#include <optional>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include "spdlog/fmt/bin_to_hex.h"

#include <opencv2/opencv.hpp>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" // Makes dumb warnings go away.
#endif
#include <libpixyusb2.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <Eigen/Eigen>
#include <gcem.hpp>
#include <networktables/NetworkTableInstance.h>

#include "cammath.hpp"
#include "transform.hpp"
#include "conversion.hpp"

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

const 

// The camera.
const camera3<double> CAMERA(Eigen::Vector2<double>(79.0, 52.0), deg2rad(60.0), Eigen::Vector3<double>(0.0, 0.0, 0.0), Eigen::Quaterniond(Eigen::AngleAxis<double>(CAM_DOWNPITCH, Eigen::Vector3d::UnitX()))); // 79x52 for line tracking according to https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:line_api#fn__3, fov of 60 degress according to https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:overview
// The plane representing the floor.
const Eigen::Hyperplane<double, 3> FLOOR(Eigen::Vector3<double>(0.0, 0.0, 1.0), Eigen::Vector3<double>(0.0, 0.0, -CAM_HEIGHT)); 

const uint32_t PIXY_FRONT_ID = 0xE4E35363; // UID of front camera.
const uint32_t PIXY_REAR_ID = 0xF1435B59; // UID of rear camera.

/**
 * Listens for NetworkTable connection events, and waits for a connection.
 */
class ConnectionWaiter {
public:
    // The listener that is registered to NetworkTables.
    struct Listener {
        // The promise of a connection.
        std::shared_ptr<std::promise<void>> p;
        // Has the promise already been fufilled?
        std::shared_ptr<bool> is_notified;

        // Default constructor.
        Listener() : p(new std::promise<void>()), is_notified(new bool(false)) {} 
        // Copy constructor.
        Listener(const Listener& l) : p(l.p), is_notified(l.is_notified) {}
        // Move constructor.
        Listener(Listener&& l) : p(std::move(l.p)), is_notified(std::move(l.is_notified)) {}

        // Called by NetworkTables on a connection change event.
        void operator()(const nt::ConnectionNotification& event) {
            spdlog::debug("NT Listener: connected: {}", event.connected);
            if(event.connected) { // Is this a connection?
                if(!*is_notified) // If we haven't notified...
                {
                    p->set_value(); // Fufill the promise.
                    *is_notified = true; // Don't do this again next time.
                }
            }
        }
    };

    Listener listener; // The listener we will use.
    std::future<void> connected; // Represents the time in the future when the connection happens.

    // Default constructor; Inits future from promise.
    ConnectionWaiter() : listener(), connected(listener.p->get_future()) {}

    // Waits for a connection, with a timeout.
    template<typename T>
    void wait_for_connection(T timeout) {
        if(!connected.valid()) { // Make sure our future is ok.
            throw std::logic_error("Future is not valid!");
        }
        if(connected.wait_for(timeout) == std::future_status::timeout) { // Wait, and check for timeout.
            throw std::runtime_error("Timeout while connecting to NetworkTables.");
        }
        return connected.get(); // Clear future.
    }

    void wait_for_connection() {
        if(!connected.valid()) { // Make sure our future is ok.
            throw std::logic_error("Future is not valid!");
        }
        return connected.get(); // Wait.
    }
};

/**
 * Manages Pixys connected to the device.
 */
class PixyFinder {
    std::mutex m_update;
    std::condition_variable do_update;
    uint32_t id_to_update;

    std::mutex m; // Mutex for the map.
    std::condition_variable update_finished;
    std::unordered_map<uint32_t, std::shared_ptr<Pixy2>> pixys; // The pixies we have.
public:
    void do_a_update() {
        std::unique_lock lock(m_update);
        do_update.wait(lock);
        spdlog::debug("Updating pixy {0:x}", id_to_update);
        
        if(pixys.count(id_to_update) > 0) {
            if(pixys.at(id_to_update).use_count() != 1) {
                spdlog::critical("Pixy update requested, but it is in use!");
                std::terminate();
            }
            pixys.erase(id_to_update);
        }
        
        while(true) {
            std::shared_ptr<Pixy2> pixy(new Pixy2());
            int res = pixy->init();
	        if(res < 0) {
		        spdlog::warn("Pixy {} not found!", id_to_update);
                break;
            }

	        uint32_t uid = 0;
            res = pixy->m_link.callChirp("getUID", END_OUT_ARGS, &uid, END_IN_ARGS); // Get UID.
            if (res < 0) {
		        spdlog::warn("Cannot get Pixy id, code {}", res);
                break;
            }
            if(uid == id_to_update)
            {
                std::unique_lock lock2(m);
                pixys.insert(std::make_pair((uint32_t)uid, std::shared_ptr(pixy)));
                spdlog::debug("Found!");
                break;
            }
        }

        update_finished.notify_all();
        
        return;
    }

    // Tries to get a Pixy by id.
    std::optional<std::shared_ptr<Pixy2>> get(uint32_t id) {
        std::unique_lock lock(m); // Lock the mutex.
        
        while(pixys.count(id) == 0) { // If the pixy is missing...
            return {}; // Return nothing.
        }
        return pixys.at(id);
    }

    void update(uint32_t id) {
        {
            std::unique_lock id_lock(m_update);
            id_to_update = id;
            id_lock.unlock();
        }
        do_update.notify_one();
        std::unique_lock lock(m);
        update_finished.wait(lock);
        return;
    }
};

// Processes vectors from a pixy and puts results in NetworkTables.
void thread_fn(std::shared_ptr<PixyFinder> p, std::shared_ptr<nt::NetworkTable> table, uint32_t id) {
    while(true) { // Do this forever.
        std::shared_ptr<Pixy2> pixy;
        
        {
            auto pixy_try = p->get(id); // Try to get pixy.
            if(!pixy_try.has_value()) { // Failed?
                table->PutBoolean("Lock", false);
                table->PutBoolean("Ok", false);
                table->GetInstance().Flush();
                p->update(id);
                continue; // Skip loop.
            }
            pixy = pixy_try.value(); // Retrieve pixy.
        } // pixy_try is deleted here.

        pixy->m_link.stop();

        uint8_t* bayer = nullptr;
        pixy->m_link.getRawFrame(&bayer);

        cv::Mat m_bayer(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8U, bayer);

        cv::Mat frame, frame_hsv;
        cv::cvtColor(m_bayer, frame, CV_BayerBG2RGB, -1);
        cv::cvtColor(frame, frame_hsv, CV_RGB2HSV, -1);

        cv::Mat green;
        cv::inRange(frame_hsv, green, cv::Scalar(70, 0, 0), cv::Scalar(90, 255, 255));

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(green, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));

        std::vector<cv::RotatedRect> rectangles;
        for(const auto c : contours) {
            rectangles.push_back(cv::minAreaRect(c));
        }

        std::sort(rectangles.begin(), rectangles.end(), [](const auto& a, const auto& b) {return a.size.area() < b.size.area();});
        retangles.resize(2);



        auto rect = cv::minAreaRect();

        double turn = 0; // TODO 
        double strafe = 0; // TODO

        table->PutNumber("Turn", turn);
        table->PutNumber("Strafe", strafe);

        bool lock = false;
        // TODO
        table->PutBoolean("Lock", lock);
        table->PutBoolean("Ok", true);
    }
}

int main() {
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_sink_mt>();
        console_sink->set_level(spdlog::level::debug);
        console_sink->set_pattern("[%^%l%$] %v");

        std::filesystem::create_directory("log");

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("log/Vision.txt", 1048576 * 5, 3);
        file_sink->set_level(spdlog::level::trace);
        file_sink->set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");
        std::vector<std::shared_ptr<spdlog::sinks::sink> > sinks = {&console_sink, &file_sink};
        auto logger = std::shared_ptr<spdlog::logger>(new spdlog::logger("Vision", sinks.begin(), sinks.end()));
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
    }

    spdlog::info("Vision Starting...");

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

    // Thread to run updates.
    std::thread thread_pixyfinder([p]() {
        while(true)
        {
            p->do_a_update();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

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
