#include "controller/app.hpp"
#include "controller/config.hpp"

#include "msg/msg.hpp"

#include <csignal>
#include <cstdio>

namespace
{

controller::controller_app* g_app = nullptr;

void handle_signal(int)
{
    if (g_app != nullptr)
    {
        g_app->shutdown();
    }
}

}  // namespace

int main(int argc, char** argv)
{
    const controller::app_config cfg = controller::load_config(argc, argv);

    if (msg::init() != msg::status::ok)
    {
        std::fprintf(stderr, "msg bus init failed\n");
        return 1;
    }

    controller::controller_app app(cfg);
    g_app = &app;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::printf("ctrl started (topic_ns=%s, imu_mode=%d", cfg.ipc_prefix.c_str(),
                static_cast<int>(cfg.imu_mode));
    if (cfg.logger.stdout_block && cfg.logger.hz > 0.0f)
    {
        std::printf(", log=%.1fHz", cfg.logger.hz);
    }
    std::printf("). Focus sim window for keyboard. Ctrl+C to exit.\n");
    app.run();
    return 0;
}
