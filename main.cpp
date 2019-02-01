#include <iostream>
#include <cmath>
#include <algorithm>
#include <future>

#include <libpixyusb2.h>
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
        Listener() : p(new std::promise<void>()) {} 
        Listener(const Listener& l) : p(l.p) {}
        Listener(Listener&& l) : p(std::move(l.p)) {}

        void operator()(const nt::ConnectionNotification& event) {
            std::cout << "NT Listener: connected: " << event.connected;
            if(event.connected) {
                p->set_value();
                std::cout << ", id: " << event.conn.remote_id << ", ip: " << event.conn.remote_ip << ":" << event.conn.remote_port;
            }
            std::cout << std::endl;
        }
    };

    Listener listener;
    std::future<void> connected;

    ConnectionWaiter() : listener(), connected(listener.p->get_future()) {

    }

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

    auto pixy_f = std::async([]() {
        Pixy2 p;
        std::cout << "Pixy Setup: Connecting to PixyCam..." << std::endl;
        if(p.init() < 0) {
            throw std::runtime_error("Failed to open PixyCam.");
        }
        p.getVersion();
        p.version->print();
        std::cout << "Pixy Setup: USB Bus: " << (int)p.m_link.getUSBBus() << std::endl;
        std::cout << "Pixy Setup: USB Path: " << std::endl;
        uint8_t ports[7];
        size_t num_ports = p.m_link.getUSBPorts(ports, 7);
        for(size_t i = 0; i < num_ports; i++) {
            std::cout << i << "Pixy Setup: >> port " << (int)ports[i] << std::endl;
        }
        p.setLamp(100, 100);
        return p;
    });

    auto networktables = networktables_f.get();
    auto pixy = pixy_f.get();

    auto table = networktables.GetTable("Vision");

    while(true) {
        pixy.line.getMainFeatures(LINE_VECTOR, true);
        
        if(pixy.line.numVectors == 0) {
            continue;
        }

        auto the_vector = pixy.line.vectors[0];

        vector2<double> a, b;

        try {
            auto transformed = transform_vector(the_vector);
            a = transformed.first;
            b = transformed.second;
        } catch (std::bad_optional_access& e) {
            std::cout << "Vision: Error processing vectors. Skipping frame..." << std::endl;
            continue;
        }

        table->PutNumber("NumVectors", pixy.line.numVectors);
        table->PutNumber("VectorX1", a.x);
        table->PutNumber("VectorY1", a.y);
        table->PutNumber("VectorX2", b.x);
        table->PutNumber("VectorY2", b.y);

        double turn = std::atan2(b.y - a.y, b.x - a.x);
        vector2<double> center = (a + b) / 2.0;
        double strafe = center.x;

        table->PutNumber("Turn", turn);
        table->PutNumber("Strafe", strafe);
        networktables.Flush();
    }
}