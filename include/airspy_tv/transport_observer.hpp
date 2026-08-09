#pragma once

#include "airspy_tv/transport_stream.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

namespace airspy_tv {

struct TransportObserverConfig {
    std::size_t queue_capacity_bytes{256U << 10U};
    std::string thread_name{"ts-observer"};
};

struct TransportObserverStats {
    bool active{};
    bool failed{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity_bytes{};
    std::string error;
};

// A latest-state TS observer. Each instance owns its queue and worker; queue
// overflow is converted into a local FEC-region discontinuity so a section
// parser cannot join bytes across a dropped region. A single producer block
// may temporarily exceed the nominal capacity so large decode batches remain
// observable instead of being rejected unconditionally.
class AsyncTransportObserver {
  public:
    using Consume = std::function<void(std::span<const std::uint8_t>)>;
    using Discontinuity = std::function<void(TransportDiscontinuity)>;

    AsyncTransportObserver(Consume consume, Discontinuity discontinuity,
                           TransportObserverConfig config = {});
    ~AsyncTransportObserver() noexcept;

    AsyncTransportObserver(const AsyncTransportObserver &) = delete;
    AsyncTransportObserver &operator=(const AsyncTransportObserver &) = delete;
    AsyncTransportObserver(AsyncTransportObserver &&) = delete;
    AsyncTransportObserver &operator=(AsyncTransportObserver &&) = delete;

    bool submit(std::span<const std::uint8_t> transport_stream) noexcept;
    void notify_discontinuity(TransportDiscontinuity discontinuity) noexcept;
    void stop(bool drain = false) noexcept;

    [[nodiscard]] TransportObserverStats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
