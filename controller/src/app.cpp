#include "controller/app.hpp"
#include "controller/ecal_io.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

namespace controller
{

controller_app::controller_app(const app_config& cfg)
    : cfg_(cfg), io_(std::make_unique<ecal_io>(cfg)), actuator_(cfg_, *io_, running_), ins_(cfg_, *io_, running_),
      command_(cfg_, running_), chassis_(cfg_, running_), log_(cfg_, running_), visualizer_(cfg_, running_)
{
    if (!io_->valid())
    {
        std::fprintf(stderr, "failed to open eCAL transport (topic_ns=%s)\n", cfg_.ipc_prefix.c_str());
        running_.store(false);
    }
}

controller_app::~controller_app() = default;

void controller_app::run()
{
    while (running_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void controller_app::shutdown()
{
    running_.store(false);
}

}  // namespace controller
