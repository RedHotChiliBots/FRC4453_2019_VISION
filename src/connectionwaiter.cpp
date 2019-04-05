#include "connectionwaiter.hpp"

ConnectionWaiter::Listener::Listener(): p(new std::promise<void>()), is_notified(new bool(false)) {} 

ConnectionWaiter::Listener::Listener(const Listener& l) : p(l.p), is_notified(l.is_notified) {}

ConnectionWaiter::Listener::Listener(Listener&& l) : p(std::move(l.p)), is_notified(std::move(l.is_notified)) {}

void ConnectionWaiter::Listener::operator()(const nt::ConnectionNotification& event) {
    spdlog::debug("NT Listener: connected: {}", event.connected);
    if(event.connected) { // Is this a connection?
        if(!*is_notified) // If we haven't notified...
        {
            p->set_value(); // Fufill the promise.
            *is_notified = true; // Don't do this again next time.
        }
    }
}

/**
 * Listens for NetworkTable connection events, and waits for a connection.
 */
ConnectionWaiter::ConnectionWaiter() : listener(), connected(listener.p->get_future()) {}


void ConnectionWaiter::wait_for_connection() {
    if(!connected.valid()) { // Make sure our future is ok.
        throw std::logic_error("Future is not valid!");
    }
    return connected.get(); // Wait.
}
