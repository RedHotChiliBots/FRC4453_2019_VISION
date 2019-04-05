#include "pixyfinder.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

PixyFinder::PixyFinder(): update_lock(m_update) {}

void PixyFinder::do_updates() {
    while(true) {
        do_update.wait(update_lock);
        spdlog::debug("Updating pixy {0:x}", id_to_update);
        
        if(pixys.count(id_to_update) > 0) {
            if(pixys.at(id_to_update).use_count() != 1) {
                spdlog::critical("Pixy update requested, but it is in use!");
                std::terminate();
            }
            pixys.erase(id_to_update);
        }

        std::vector<std::shared_ptr<Pixy2> > jail;
        
        while(true) {
            spdlog::trace("Getting a pixy...");
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
                std::unique_lock<std::mutex> lock2(m);
                pixys.insert(std::make_pair((uint32_t)uid, std::shared_ptr<Pixy2>(pixy)));
                spdlog::debug("Found!");
                break;
            }
            spdlog::trace("Pixy doesn't match, was {}", uid);
            jail.push_back(pixy);
        }

        std::unique_lock<std::mutex> lock_m(m); // Wait a bit if we were too fast.
        lock_m.unlock();
        update_finished.notify_all();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

std::shared_ptr<Pixy2> PixyFinder::get(uint32_t id) {
    std::unique_lock<std::mutex> lock(m); // Lock the mutex.
    
    while(pixys.count(id) == 0) { // If the pixy is missing...
        throw std::runtime_error("Missing pixy!"); // Return nothing.
    }
    return pixys.at(id);
}

void PixyFinder::update(uint32_t id) {
    {
        std::unique_lock<std::mutex> id_lock(m_update);
        id_to_update = id;
        id_lock.unlock();
    }
    std::unique_lock<std::mutex> lock(m);
    do_update.notify_one();
    update_finished.wait(lock);
    return;
}