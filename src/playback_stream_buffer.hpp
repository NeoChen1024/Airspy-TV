#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace airspy_tv {

enum class PlaybackBufferAction { none, load, stop };

struct PlaybackBufferSnapshot {
    bool source_active{};
    bool buffering{};
    std::size_t queued_bytes{};
    std::size_t queue_capacity{};
    std::uint64_t blocks_accepted{};
    std::uint64_t bytes_accepted{};
    std::uint64_t blocks_processed{};
    std::uint64_t bytes_processed{};
    std::uint64_t dropped_blocks{};
    std::uint64_t dropped_bytes{};
    std::uint64_t discontinuities{};
};

// Internal byte-stream queue behind MpvPlayer's libmpv callback. Keeping the
// queue policy independent of libmpv makes buffering, EOF, and generation
// boundaries deterministic to test.
class PlaybackStreamBuffer {
  public:
    struct Reader {
        std::uint64_t generation{};
        std::shared_ptr<std::atomic<bool>> canceled{
            std::make_shared<std::atomic<bool>>(false)};
    };

    static constexpr std::size_t capacity = 8U << 20U;
    static constexpr std::size_t low_watermark = 1U << 20U;
    static constexpr std::size_t resume_watermark = 2U << 20U;

    PlaybackStreamBuffer();
    ~PlaybackStreamBuffer() noexcept;

    PlaybackStreamBuffer(const PlaybackStreamBuffer &) = delete;
    PlaybackStreamBuffer &operator=(const PlaybackStreamBuffer &) = delete;

    [[nodiscard]] Reader open_reader() const;
    [[nodiscard]] std::size_t read(Reader &reader,
                                   std::span<std::uint8_t> output);
    void cancel(Reader &reader) noexcept;

    [[nodiscard]] PlaybackBufferAction set_source_active(bool active);
    [[nodiscard]] PlaybackBufferAction restart();
    void submit(std::span<const std::uint8_t> bytes);
    void fec_region_reset() noexcept;
    void stream_end() noexcept;
    [[nodiscard]] PlaybackBufferAction retune();
    void shutdown() noexcept;

    [[nodiscard]] PlaybackBufferSnapshot snapshot() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace airspy_tv
