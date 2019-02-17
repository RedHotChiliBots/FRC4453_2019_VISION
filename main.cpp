#include <iostream>
#include <cmath>
#include <algorithm>
#include <future>
#include <sstream>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" // Makes dumb warnings go away.
#endif
#include <libpixyusb2.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <gcem.hpp>
#include <networktables/NetworkTableInstance.h>

#include "geom.hpp"
#include "cammath.hpp"

constexpr double rad2deg(double v) {
    return v * (180.0 / M_PI);
}

constexpr double deg2rad(double v) {
    return v / (180.0 / M_PI);
}

constexpr double CAM_HEIGHT = 4.0;
constexpr double CAM_DOWNPITCH = deg2rad(-40.0);

const camera3<double> CAMERA(vector2<double>(79.0, 52.0), deg2rad(60.0), vector3<double>(0.0, 0.0, 0.0), vector3<double>(CAM_DOWNPITCH, 0.0, 0.0)); // 79x52 for line tracking according to https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:line_api#fn__3, fov of 60 degress according to https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:overview
constexpr plane3<double> FLOOR(vector3<double>(0.0, 0.0, -CAM_HEIGHT), vector3<double>(0.0, 0.0, 1.0).normalize()); 

const uint32_t PIXY_FRONT_ID = 0xE4E35363;
const uint32_t PIXY_REAR_ID = 0xF1435B59;

std::pair<vector2<double>, vector2<double>> transform_vector(const Vector& v) {
    vector2<double> a((double)v.m_x0, (double)v.m_y0), b((double)v.m_x1, (double)v.m_y1);

    auto a_r = CAMERA.cast_ray(a);
    auto b_r = CAMERA.cast_ray(b);

    return std::make_pair(FLOOR.ray_intersect(a_r).value().truncate(), FLOOR.ray_intersect(b_r).value().truncate());
}

class ConnectionWaiter {
public:
    struct Listener {
        std::shared_ptr<std::promise<void>> p;
        std::shared_ptr<bool> is_notified;
        Listener() : p(new std::promise<void>()), is_notified(new bool(false)) {} 
        Listener(const Listener& l) : p(l.p), is_notified(l.is_notified) {}
        Listener(Listener&& l) : p(std::move(l.p)), is_notified(std::move(l.is_notified)) {}


        void operator()(const nt::ConnectionNotification& event) {
            std::cout << "NT Listener: connected: " << event.connected;
            if(event.connected) {
                if(!*is_notified)
                {
                    p->set_value();
                    *is_notified = true;
                }
                std::cout << ", id: " << event.conn.remote_id << ", ip: " << event.conn.remote_ip << ":" << event.conn.remote_port;
            }
            std::cout << std::endl;
        }
    };

    Listener listener;
    std::future<void> connected;

    ConnectionWaiter() : listener(), connected(listener.p->get_future()) {}

    template<typename T>
    void wait_for_connection(T timeout) {
        if(!connected.valid()) {
            throw std::logic_error("Future is not valid!");
        }
        if(connected.wait_for(timeout) == std::future_status::timeout) {
            throw std::runtime_error("Timeout while connecting to NetworkTables.");
        }
        return connected.get();
    }
};

class PixyFinder {
    std::unordered_map<uint32_t, std::shared_ptr<Pixy2>> pixys;
public:
    size_t enumerate() {
	std::cout << "Enumerating Pixys..." << std::endl;
        while(true) {
            std::shared_ptr<Pixy2> pixy(new Pixy2());
            int res = pixy->init();
	        if(res < 0) {
		        std::cout << "Done! Code: " << res << std::endl;
                break;
            }

            std::vector<uint8_t> id(8);

	        uint32_t uid = 0;
            res = pixy->m_link.callChirp("getUID", END_OUT_ARGS, &uid, END_IN_ARGS);
            if (res < 0) {
		        std::cout << "Done! Code: " << res << std::endl;
                break;
            }
            pixys.insert(std::make_pair((uint32_t)uid, std::shared_ptr(pixy)));
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

    std::shared_ptr<Pixy2> get(uint32_t id) {
        return pixys.at(id);
    }
};

void thread_fn(std::shared_ptr<Pixy2> pixy, std::shared_ptr<nt::NetworkTable> table) {
    while(true) {
        pixy->line.getMainFeatures(LINE_VECTOR, true);
        
        if(pixy->line.numVectors > 0) {
            auto the_vector = pixy->line.vectors[0];

            vector2<double> a, b;

            try {
                auto transformed = transform_vector(the_vector);
                a = transformed.first;
                b = transformed.second;
            } catch (std::bad_optional_access& e) {
                std::cout << "Vision: Error processing vectors. Skipping frame..." << std::endl;
                continue;
            }

//          table->PutNumber("VectorX1", a.x);
//          table->PutNumber("VectorY1", a.y);
//          table->PutNumber("VectorX2", b.x);
//          table->PutNumber("VectorY2", b.y);

            double turn = rad2deg(std::atan2(b.y - a.y, b.x - a.x)) + 90;
            vector2<double> center = (a + b) / 2.0;
            double strafe = center.x;

            table->PutNumber("Turn", turn);
            table->PutNumber("Strafe", strafe);
            table->GetInstance().Flush();
        }

        table->PutNumber("NumVectors", pixy->line.numVectors);
    }
}

int main() {
    auto networktables_f = std::async([]() {
        std::cout << "NT Setup: Connecting to NetworkTables..." << std::endl;
        ConnectionWaiter waiter;
        auto nt_inst = nt::NetworkTableInstance::GetDefault();
        nt_inst.SetNetworkIdentity("Vision");
        nt_inst.AddConnectionListener(waiter.listener, true);
        nt_inst.StartClientTeam(4453);
        waiter.wait_for_connection(std::chrono::seconds(10));
        std::cout << "NT Setup: Connected to NetworkTables." << std::endl;
        return nt_inst;
    });

    auto pixys_f = std::async([]() {
        PixyFinder p;
        p.enumerate();
        return p;
    });

    std::shared_ptr<Pixy2> front_pixy;
    std::shared_ptr<Pixy2> rear_pixy;

    {
        auto pixys = pixys_f.get();
        front_pixy = pixys.get(PIXY_FRONT_ID);
        rear_pixy = pixys.get(PIXY_REAR_ID);
    }

    auto networktables = networktables_f.get();

    auto table = networktables.GetTable("Vision");

    auto front_table = table->GetSubTable("Front");
    auto rear_table = table->GetSubTable("Rear");

    std::thread thread_front(thread_fn, front_pixy, front_table);
    std::thread thread_rear(thread_fn, rear_pixy, rear_table);

    thread_front.join();
    thread_rear.join();
}
