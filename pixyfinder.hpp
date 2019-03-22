#ifndef PIXYFINDER_HPP
#define PIXYFINDER_HPP
#include <mutex>
#include <memory>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic" // Makes dumb warnings go away.
#endif
#include <libpixyusb2.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

/**
 * Manages Pixys connected to the device.
 */
class PixyFinder {
    std::mutex m_update;
    std::unique_lock<std::mutex> update_lock;
    std::condition_variable do_update;
    uint32_t id_to_update;

    std::mutex m; // Mutex for the map.
    std::condition_variable update_finished;
    std::unordered_map<uint32_t, std::shared_ptr<Pixy2>> pixys; // The pixies we have.
public:
    PixyFinder();
    
    void do_updates();

    // Tries to get a Pixy by id.
    std::shared_ptr<Pixy2> get(uint32_t id);

    void update(uint32_t id);
};


#endif