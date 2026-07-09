#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <type_traits>
#include <unistd.h>

namespace robot_ipc
{

enum class channel_status
{
    ok = 0,
    error = -1,
    timeout = -2,
    invalid = -3,
};

template <typename T>
class ShmChannel
{
public:
    static_assert(std::is_trivially_copyable_v<T>, "ShmChannel payload must be trivially copyable");

    ShmChannel() = default;
    ShmChannel(const ShmChannel&) = delete;
    ShmChannel& operator=(const ShmChannel&) = delete;
    ShmChannel(ShmChannel&& other) noexcept { *this = std::move(other); }
    ShmChannel& operator=(ShmChannel&& other) noexcept
    {
        if (this != &other)
        {
            reset();
            block_ = other.block_;
            data_ = other.data_;
            fd_ = other.fd_;
            owner_ = other.owner_;
            other.block_ = nullptr;
            other.data_ = nullptr;
            other.fd_ = -1;
            other.owner_ = false;
        }
        return *this;
    }

    ~ShmChannel() { reset(); }

    static ShmChannel open_server(const std::string& name) { return open(name, true); }
    static ShmChannel open_client(const std::string& name) { return open(name, false); }

    [[nodiscard]] bool valid() const { return block_ != nullptr && data_ != nullptr; }

    channel_status write(const T& value)
    {
        if (!valid())
        {
            return channel_status::invalid;
        }

        pthread_mutex_lock(&block_->header.mutex);
        std::memcpy(&block_->payload, &value, sizeof(T));
        ++block_->header.sequence;
        block_->header.valid = 1;
        pthread_mutex_unlock(&block_->header.mutex);
        return channel_status::ok;
    }

    channel_status read(T& value, bool wait = false, int timeout_ms = 1000)
    {
        if (!valid())
        {
            return channel_status::invalid;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

        while (true)
        {
            pthread_mutex_lock(&block_->header.mutex);
            if (block_->header.valid != 0)
            {
                std::memcpy(&value, &block_->payload, sizeof(T));
                pthread_mutex_unlock(&block_->header.mutex);
                return channel_status::ok;
            }
            pthread_mutex_unlock(&block_->header.mutex);

            if (!wait)
            {
                return channel_status::error;
            }

            if (std::chrono::steady_clock::now() >= deadline)
            {
                return channel_status::timeout;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void invalidate()
    {
        if (!valid())
        {
            return;
        }

        pthread_mutex_lock(&block_->header.mutex);
        block_->header.valid = 0;
        pthread_mutex_unlock(&block_->header.mutex);
    }

private:
    static constexpr std::uint32_t kMagic = 0x57425231u;

    struct SharedHeader
    {
        std::uint32_t magic = 0;
        pthread_mutex_t mutex{};
        std::uint64_t sequence = 0;
        std::uint32_t valid = 0;
        std::uint32_t _pad = 0;
    };

    struct SharedBlock
    {
        SharedHeader header;
        alignas(T) T payload{};
    };

    SharedBlock* block_ = nullptr;
    T* data_ = nullptr;
    int fd_ = -1;
    bool owner_ = false;

    void reset()
    {
        if (block_ != nullptr)
        {
            munmap(block_, sizeof(SharedBlock));
            block_ = nullptr;
            data_ = nullptr;
        }
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
        owner_ = false;
    }

    static std::string shm_path(const std::string& name)
    {
        if (!name.empty() && name[0] == '/')
        {
            return name;
        }
        return "/" + name;
    }

    static bool init_mutex(pthread_mutex_t* mutex)
    {
        pthread_mutexattr_t attr;
        if (pthread_mutexattr_init(&attr) != 0)
        {
            return false;
        }
        if (pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED) != 0)
        {
            pthread_mutexattr_destroy(&attr);
            return false;
        }
        const int rc = pthread_mutex_init(mutex, &attr);
        pthread_mutexattr_destroy(&attr);
        return rc == 0;
    }

    static ShmChannel open(const std::string& name, bool create)
    {
        ShmChannel channel;
        const std::string path = shm_path(name);
        const int flags = create ? (O_CREAT | O_RDWR) : O_RDWR;
        channel.fd_ = shm_open(path.c_str(), flags, 0666);
        if (channel.fd_ < 0)
        {
            return channel;
        }

        if (create && ftruncate(channel.fd_, static_cast<off_t>(sizeof(SharedBlock))) != 0)
        {
            channel.reset();
            return channel;
        }

        channel.block_ = static_cast<SharedBlock*>(
            mmap(nullptr, sizeof(SharedBlock), PROT_READ | PROT_WRITE, MAP_SHARED, channel.fd_, 0));
        if (channel.block_ == MAP_FAILED)
        {
            channel.block_ = nullptr;
            channel.reset();
            return channel;
        }

        channel.data_ = &channel.block_->payload;
        channel.owner_ = create;

        if (create)
        {
            if (channel.block_->header.magic != kMagic)
            {
                std::memset(channel.block_, 0, sizeof(SharedBlock));
                channel.block_->header.magic = kMagic;
                if (!init_mutex(&channel.block_->header.mutex))
                {
                    channel.reset();
                    return channel;
                }
            }
        }
        else if (channel.block_->header.magic != kMagic)
        {
            channel.reset();
            return channel;
        }

        return channel;
    }
};

}  // namespace robot_ipc
