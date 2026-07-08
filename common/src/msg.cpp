#include "msg/msg.hpp"

#include <cstring>

namespace msg
{

namespace
{

bool bus_ready = false;

topic_layout* layout_of(const topic* topic)
{
    if (topic == nullptr || !topic->valid())
    {
        return nullptr;
    }
    return topic->layout;
}

}  // namespace

status init()
{
    if (bus_ready)
    {
        return status::ok;
    }

    bus_ready = true;
    return status::ok;
}

subscriber subscribe(const topic* topic)
{
    auto* layout = layout_of(topic);
    if (layout == nullptr)
    {
        return subscriber::invalid();
    }

    std::unique_lock<std::mutex> guard(layout->lock);
    subscriber sub = subscriber::invalid();
    for (std::uint8_t i = 0; i < max_subscribers_per_topic; ++i)
    {
        if (!layout->subs[i].active)
        {
            layout->subs[i].active = true;
            layout->subs[i].pending = false;
            ++layout->sub_count;
            sub.layout = layout;
            sub.slot_idx = i;
            break;
        }
    }

    return sub;
}

status publish(const topic* topic, const void* data, std::size_t size, publish_opts opts)
{
    if (!bus_ready || data == nullptr || size == 0)
    {
        return status::invalid_arg;
    }

    auto* layout = layout_of(topic);
    if (layout == nullptr || layout->payload_bytes == nullptr)
    {
        return status::invalid_arg;
    }

    if (size != layout->payload_size)
    {
        return status::invalid_arg;
    }

    std::unique_lock<std::mutex> guard(layout->lock, std::defer_lock);
    if (opts.block)
    {
        guard.lock();
    }
    else if (!guard.try_lock())
    {
        return status::error;
    }

    std::memcpy(layout->payload_bytes, data, size);

    for (auto& slot : layout->subs)
    {
        if (slot.active)
        {
            slot.pending = true;
        }
    }

    return status::ok;
}

bool available(subscriber sub)
{
    if (!bus_ready || !sub.valid())
    {
        return false;
    }

    if (!sub.layout->created)
    {
        return false;
    }

    std::lock_guard<std::mutex> guard(sub.layout->lock);
    const auto& slot = sub.layout->subs[sub.slot_idx];
    return slot.active && slot.pending;
}

status read(subscriber sub, void* buf, std::size_t size)
{
    if (!bus_ready || !sub.valid() || buf == nullptr || size == 0)
    {
        return status::invalid_arg;
    }

    auto* layout = sub.layout;
    if (!layout->created || layout->payload_bytes == nullptr)
    {
        return status::invalid_arg;
    }

    auto& slot = layout->subs[sub.slot_idx];
    if (!slot.active)
    {
        return status::invalid_arg;
    }

    if (size != layout->payload_size)
    {
        return status::invalid_arg;
    }

    std::lock_guard<std::mutex> guard(layout->lock);
    if (!slot.pending)
    {
        return status::error;
    }

    std::memcpy(buf, layout->payload_bytes, layout->payload_size);
    slot.pending = false;
    return status::ok;
}

}  // namespace msg
