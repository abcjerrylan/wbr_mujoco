#pragma once

#include "msg/status.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <type_traits>

namespace msg
{

inline constexpr std::uint8_t max_subscribers_per_topic = 8;

struct subscriber_slot
{
    bool active = false;
    bool pending = false;
};

struct topic_layout
{
    std::mutex lock{};
    std::size_t payload_size = 0;
    std::uint8_t* payload_bytes = nullptr;
    subscriber_slot subs[max_subscribers_per_topic]{};
    std::uint8_t sub_count = 0;
    bool created = false;
};

template <typename Payload>
struct topic_entry : topic_layout
{
    alignas(Payload) Payload payload{};
};

struct topic
{
    topic_layout* layout = nullptr;

    [[nodiscard]] bool valid() const { return layout != nullptr && layout->created; }
};

struct publish_opts
{
    bool block = true;
};

struct subscriber
{
    topic_layout* layout = nullptr;
    std::uint8_t slot_idx = 0xFF;

    [[nodiscard]] bool valid() const { return layout != nullptr && slot_idx != 0xFF; }

    [[nodiscard]] static constexpr subscriber invalid() { return {nullptr, 0xFF}; }
};

status init();

template <typename Payload>
topic* create()
{
    static_assert(std::is_trivially_copyable_v<Payload>, "msg payload must be trivially copyable");

    if (init() != status::ok)
    {
        return nullptr;
    }

    static topic_entry<Payload> storage;
    static topic handle;

    if (!storage.created)
    {
        storage.payload_size = sizeof(Payload);
        storage.payload_bytes = reinterpret_cast<std::uint8_t*>(&storage.payload);
        storage.sub_count = 0;

        for (auto& slot : storage.subs)
        {
            slot.active = false;
            slot.pending = false;
        }

        storage.created = true;
        handle.layout = &storage;
    }

    return &handle;
}

subscriber subscribe(const topic* topic);

template <typename Payload>
subscriber subscribe()
{
    const topic* topic = create<Payload>();
    if (topic == nullptr)
    {
        return subscriber::invalid();
    }
    return subscribe(topic);
}

status publish(const topic* topic, const void* data, std::size_t size, publish_opts opts = {});

template <typename T>
status publish(const T& data, publish_opts opts = {})
{
    static_assert(std::is_trivially_copyable_v<T>, "msg payload must be trivially copyable");
    topic* topic = create<T>();
    if (topic == nullptr)
    {
        return status::error;
    }
    return publish(topic, data, opts);
}

template <typename T>
status publish(const topic* topic, const T& data, publish_opts opts = {})
{
    static_assert(std::is_trivially_copyable_v<T>, "msg payload must be trivially copyable");
    return publish(topic, &data, sizeof(T), opts);
}

bool available(subscriber sub);
status read(subscriber sub, void* buf, std::size_t size);

template <typename T>
status read(subscriber sub, T& out)
{
    static_assert(std::is_trivially_copyable_v<T>, "msg payload must be trivially copyable");
    return read(sub, &out, sizeof(T));
}

}  // namespace msg
