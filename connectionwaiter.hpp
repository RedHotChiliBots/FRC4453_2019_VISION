#ifndef CONNECTIONWAITER_HPP
#define CONNECTIONWAITER_HPP
#include <future>
#include <exception>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <networktables/NetworkTableInstance.h>


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
        Listener(); 
        // Copy constructor.
        Listener(const Listener& l);
        // Move constructor.
        Listener(Listener&& l);

        // Called by NetworkTables on a connection change event.
        void operator()(const nt::ConnectionNotification& event);
    };

    Listener listener; // The listener we will use.
    std::future<void> connected; // Represents the time in the future when the connection happens.

    // Default constructor; Inits future from promise.
    ConnectionWaiter();

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

    void wait_for_connection();
};


#endif