#pragma once

#include "controller/config.hpp"
#include "controller/services.hpp"

#include <atomic>
#include <memory>

namespace controller
{

class ecal_io;

class controller_app
{
public:
    explicit controller_app(const app_config& cfg);
    ~controller_app();
    void run();
    void shutdown();

private:
    app_config cfg_;
    std::atomic<bool> running_{true};
    std::unique_ptr<ecal_io> io_;
    actuator_service actuator_;
    ins_service ins_;
    command_service command_;
    chassis_service chassis_;
    sim_log_service log_;
};

}  // namespace controller
