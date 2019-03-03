#define _USE_MATH_DEFINES // Needed on MSVC for M_PI.
#include <cmath>
#include <iostream>
#include <algorithm>
#include <future>
#include <sstream>
#include <optional>

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

constexpr double LOCK_MAX_DIST = 60.0; // Max distance to valid line in inches.
constexpr double LOCK_MIN_LENGTH = 8.0; // Minimum length of valid line in inches.

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
            std::cout << "NT Listener: connected: " << event.connected;
            if(event.connected) { // Is this a connection?
                if(!*is_notified) // If we haven't notified...
                {
                    p->set_value(); // Fufill the promise.
                    *is_notified = true; // Don't do this again next time.
                }
                std::cout << ", id: " << event.conn.remote_id << ", ip: " << event.conn.remote_ip << ":" << event.conn.remote_port;
            }
            std::cout << std::endl;
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
    std::mutex m; // Mutex for the map.
    std::unordered_map<uint32_t, std::shared_ptr<Pixy2>> pixys; // The pixies we have.
    std::condition_variable do_update; // Notifier that requests an update of pixys.
public:
    // Waits for an update request, and looks for more pixies.
    size_t enumerate() {
	    std::unique_lock lock(m); // Lock the mutex.
        do_update.wait(lock); // Wait for a request; implicitly unlocks mutex while waiting and relocks it after.
        std::cout << "Enumerating Pixys..." << std::endl;
        while(true) {
            std::shared_ptr<Pixy2> pixy(new Pixy2()); // Create a Pixy.
            int res = pixy->init(); // Try to init it.
	        if(res < 0) { // Failed?
		        std::cout << "Done! Code: " << res << std::endl;
                break; // Were done, exit loop.
            }

	        uint32_t uid = 0;
            res = pixy->m_link.callChirp("getUID", END_OUT_ARGS, &uid, END_IN_ARGS); // Get UID.
            if (res < 0) { // Failed?
		        std::cout << "Done! Code: " << res << std::endl;
                break; // Were done, exit loop.
            }
            if(pixys.count(uid) == 0) // If this pixy is new (should always be true).
            {
                pixys.insert(std::make_pair((uint32_t)uid, std::shared_ptr(pixy))); // Add pixy to our map.
            }
            std::cout << "Found!" << std::endl;
        }

//#ifndef NDEBUG
        std::cout << "Pixy ids: " << std::endl;
        for(const auto& i : pixys) {
            std::cout << ">> " << std::hex << i.first << std::dec << std::endl;
        }
//#endif
        return pixys.size();
    }

    // Tries to get a Pixy by id. If it is missing, requests an update.
    std::optional<std::shared_ptr<Pixy2>> get(uint32_t id) {
        std::unique_lock lock(m); // Lock the mutex.
        
        while(pixys.count(id) == 0) { // If the pixy is missing...
            lock.unlock(); // Let enumerate do its work.
            do_update.notify_one(); // Request update.
            return {}; // Return nothing.
        }

        return pixys.at(id);
    }

    // Requests an update, and waits for it to finish.
    void update() {
        std::unique_lock lock(m); // Lock mutex, makes sure an updates not already happening.
        lock.unlock();
        do_update.notify_one(); // Request update.
        lock.lock(); // Wait for completion.
        return;
    }
    
    // Removes a Pixy if it exists, and updates.
    void update_one(uint32_t id) {
        std::unique_lock lock(m);
        
        if(pixys.count(id) > 0) // If the pixy exists...
        {
            auto p = pixys.at(id); // Get it.
            pixys.erase(id); // Delete it from the map.
            if(!p.unique()) { // Are we the only one who has it?
                // No, thats an error.
                throw std::logic_error("Tried to update Pixy that is still being used!");
            }
        } // Pixy is implicitly deleted here.

        lock.unlock();
        do_update.notify_one(); // Request update.
        lock.lock(); // Wait for completion.
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
                std::this_thread::yield();
                continue; // Skip loop.
            }
            pixy = pixy_try.value(); // Retrieve pixy.
        } // pixy_try is deleted here.

        if (pixy->line.getMainFeatures(LINE_VECTOR, true) < 0) { // Try to get vectors.
            table->PutBoolean("Lock", false); // Failed.
            table->PutBoolean("Ok", false);
            table->GetInstance().Flush();
            pixy.reset(); // Let go of pixy, needed to update.
            p->update_one(id); // Update pixy.
            continue; // Skip loop.
        }

        if (pixy->line.numVectors > 0) { // If we have some vectors...
            auto the_vector = pixy->line.vectors[0]; // Get the best one.

            Eigen::Vector2<double> a, b;

            // Do math to make vector into floor coordinates.
            auto transformed = transform_vector<double>(pixy_vec_to_vec2<double>(the_vector), CAMERA, FLOOR);
            a = transformed.first; // Save results.
            b = transformed.second;
            
            #ifdef DEV_TEST
            table->PutNumber("VectorX1", a.x());
            table->PutNumber("VectorY1", a.y());
            table->PutNumber("VectorX2", b.x());
            table->PutNumber("VectorY2", b.y());
            #endif

            // Calculate vector angle.
            double turn = rad2deg(std::atan2(b.y() - a.y(), b.x() - a.x())); 
            Eigen::Vector2<double> center = (a + b) / 2.0; // Get center of vector.
            double strafe = center.x();

            if(id == PIXY_REAR_ID) {
                strafe += CAM_REAR_OFFSET; // Add rear offset.
            }

            table->PutNumber("Turn", turn);
            table->PutNumber("Strafe", strafe);

            bool lock;
            {
                double dist = center.norm(); // Calc distance of vector.
                auto v = b - a;
                double len = v.norm(); // Calc length of vector.

                // Is this vector good enough?
                lock = dist < LOCK_MAX_DIST && len > LOCK_MIN_LENGTH;
            }

            table->PutBoolean("Lock", lock);
            table->PutBoolean("Ok", true);
            table->GetInstance().Flush();
        }

        table->PutNumber("NumVectors", pixy->line.numVectors);
    }
}

int main() {

    // In another thread, setup NetworkTables.
    auto networktables_f = std::async([]() {
        std::cout << "NT Setup: Connecting to NetworkTables..." << std::endl;
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
        std::cout << "NT Setup: Connected to NetworkTables." << std::endl;
        return nt_inst;
    });

    std::shared_ptr<PixyFinder> p(new PixyFinder()); // Create PixyFinder.

    // Thread to run updates.
    std::thread thread_pixyfinder([p]() {
        while(true)
        {
            p->enumerate();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    p->update(); // Run first update.

    auto networktables = networktables_f.get(); // Wait for NT setup, get result.

    auto table = networktables.GetTable("Vision");

    auto front_table = table->GetSubTable("Front");
    auto rear_table = table->GetSubTable("Rear");

    // Thread for front camera.
    std::thread thread_front(thread_fn, p, front_table, PIXY_FRONT_ID);
    // Thread for rear camera.
    std::thread thread_rear(thread_fn, p, rear_table, PIXY_REAR_ID);

    // Wait for threads (which should never exit).
    thread_front.join();
    thread_rear.join();
    thread_pixyfinder.join();
}
