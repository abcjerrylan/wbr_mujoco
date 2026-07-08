// Minimal MuJoCo viewer: load robot scene from XML (unitree_mujoco-style structure).

#define private public
#include "glfw_adapter.h"
#undef private

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "array_safety.h"
#include "simulate.h"

namespace
{
    namespace mj = ::mujoco;
    namespace mju = ::mujoco::sample_util;

    const double syncMisalign = 0.1;
    const double simRefreshFraction = 0.7;
    const int kErrorLength = 1024;

    mjModel *m = nullptr;
    mjData *d = nullptr;
    mjtNum *ctrlnoise = nullptr;

    using Seconds = std::chrono::duration<double>;

    std::string ResolveScenePath(int argc, char **argv)
    {
        if (argc > 1)
        {
            return argv[1];
        }

        const std::filesystem::path candidates[] = {
            "mjcf/scene.xml",
            "../mjcf/scene.xml",
            "../../mjcf/scene.xml",
        };

        for (const auto &candidate : candidates)
        {
            if (std::filesystem::exists(candidate))
            {
                return std::filesystem::absolute(candidate).string();
            }
        }

        return "mjcf/scene.xml";
    }

    mjModel *LoadModel(const char *file, mj::Simulate &sim)
    {
        char filename[mj::Simulate::kMaxFilenameLength];
        mju::strcpy_arr(filename, file);

        if (!filename[0])
        {
            return nullptr;
        }

        char loadError[kErrorLength] = "";
        mjModel *mnew = nullptr;

        if (mju::strlen_arr(filename) > 4 &&
            !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                          mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
        {
            mnew = mj_loadModel(filename, nullptr);
            if (!mnew)
            {
                mju::strcpy_arr(loadError, "could not load binary model");
            }
        }
        else
        {
            mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
            if (loadError[0])
            {
                int error_length = mju::strlen_arr(loadError);
                if (loadError[error_length - 1] == '\n')
                {
                    loadError[error_length - 1] = '\0';
                }
            }
        }

        mju::strcpy_arr(sim.load_error, loadError);

        if (!mnew)
        {
            std::printf("%s\n", loadError);
            return nullptr;
        }

        if (loadError[0])
        {
            std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
            sim.run = 0;
        }

        return mnew;
    }

    void PhysicsLoop(mj::Simulate &sim)
    {
        std::chrono::time_point<mj::Simulate::Clock> syncCPU;
        mjtNum syncSim = 0;

        while (!sim.exitrequest.load())
        {
            if (sim.droploadrequest.load())
            {
                sim.LoadMessage(sim.dropfilename);
                mjModel *mnew = LoadModel(sim.dropfilename, sim);
                sim.droploadrequest.store(false);

                mjData *dnew = nullptr;
                if (mnew)
                {
                    dnew = mj_makeData(mnew);
                }
                if (dnew)
                {
                    sim.Load(mnew, dnew, sim.dropfilename);

                    mj_deleteData(d);
                    mj_deleteModel(m);

                    m = mnew;
                    d = dnew;
                    mj_forward(m, d);

                    free(ctrlnoise);
                    ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
                    mju_zero(ctrlnoise, m->nu);
                }
                else
                {
                    sim.LoadMessageClear();
                }
            }

            if (sim.uiloadrequest.load())
            {
                sim.uiloadrequest.fetch_sub(1);
                sim.LoadMessage(sim.filename);
                mjModel *mnew = LoadModel(sim.filename, sim);
                mjData *dnew = nullptr;
                if (mnew)
                {
                    dnew = mj_makeData(mnew);
                }
                if (dnew)
                {
                    sim.Load(mnew, dnew, sim.filename);

                    mj_deleteData(d);
                    mj_deleteModel(m);

                    m = mnew;
                    d = dnew;
                    mj_forward(m, d);

                    free(ctrlnoise);
                    ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
                    mju_zero(ctrlnoise, m->nu);
                }
                else
                {
                    sim.LoadMessageClear();
                }
            }

            if (sim.run && sim.busywait)
            {
                std::this_thread::yield();
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            {
                const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

                if (m)
                {
                    if (sim.run)
                    {
                        bool stepped = false;
                        const auto startCPU = mj::Simulate::Clock::now();
                        const auto elapsedCPU = startCPU - syncCPU;
                        double elapsedSim = d->time - syncSim;

                        if (sim.ctrl_noise_std)
                        {
                            mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
                            mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

                            for (int i = 0; i < m->nu; i++)
                            {
                                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);
                                d->ctrl[i] = ctrlnoise[i];
                            }
                        }

                        double slowdown = 100 / sim.percentRealTime[sim.real_time_index];
                        bool misaligned =
                            mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

                        if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                            misaligned || sim.speed_changed)
                        {
                            syncCPU = startCPU;
                            syncSim = d->time;
                            sim.speed_changed = false;
                            mj_step(m, d);
                            stepped = true;
                        }
                        else
                        {
                            bool measured = false;
                            mjtNum prevSim = d->time;
                            double refreshTime = simRefreshFraction / sim.refresh_rate;

                            while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
                            {
                                if (!measured && elapsedSim)
                                {
                                    sim.measured_slowdown =
                                        std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                                    measured = true;
                                }

                                mj_step(m, d);
                                stepped = true;

                                if (d->time < prevSim)
                                {
                                    break;
                                }
                            }
                        }

                        if (stepped)
                        {
                            sim.AddToHistory();
                        }
                    }
                    else
                    {
                        mj_forward(m, d);
                        sim.speed_changed = true;
                    }
                }
            }
        }
    }
} // namespace

void LoadHomeKeyframe(mj::Simulate *sim)
{
    if (!m || !d)
    {
        return;
    }

    const int key = mj_name2id(m, mjOBJ_KEY, "home");
    if (key >= 0)
    {
        mj_resetDataKeyframe(m, d, key);
        if (sim)
        {
            sim->key = key;
        }
        mj_forward(m, d);
    }
}

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
    if (filename != nullptr)
    {
        sim->LoadMessage(filename);
        m = LoadModel(filename, *sim);
        if (m)
        {
            d = mj_makeData(m);
        }
        if (d)
        {
            sim->Load(m, d, filename);
            LoadHomeKeyframe(sim);

            free(ctrlnoise);
            ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
            mju_zero(ctrlnoise, m->nu);
        }
        else
        {
            sim->LoadMessageClear();
        }
    }

    PhysicsLoop(*sim);

    free(ctrlnoise);
    mj_deleteData(d);
    mj_deleteModel(m);
}

void user_key_cb(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    (void)window;
    (void)scancode;
    (void)mods;

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE && m && d)
    {
        LoadHomeKeyframe(nullptr);
    }
}

int main(int argc, char **argv)
{
    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    const std::string scene_path = ResolveScenePath(argc, argv);
    std::printf("Loading scene: %s\n", scene_path.c_str());

    auto sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::thread physicsthreadhandle(&PhysicsThread, sim.get(), scene_path.c_str());
    glfwSetKeyCallback(static_cast<mj::GlfwAdapter *>(sim->platform_ui.get())->window_, user_key_cb);
    sim->RenderLoop();
    physicsthreadhandle.join();

    return 0;
}
