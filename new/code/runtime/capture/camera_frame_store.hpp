#ifndef LS2K_RUNTIME_CAMERA_FRAME_STORE_HPP
#define LS2K_RUNTIME_CAMERA_FRAME_STORE_HPP

#include <optional>

#include "port/camera_frame_types.hpp"
#include "runtime/runtime_state.hpp"

namespace ls2k::runtime {

/// CameraFrameStore owns latest/history semantics for gray camera frames.
class CameraFrameStore {
public:
    class ReadLease {
    public:
        ReadLease() = default;
        ReadLease(const ReadLease&) = delete;
        ReadLease& operator=(const ReadLease&) = delete;
        ReadLease(ReadLease&& other) noexcept;
        ReadLease& operator=(ReadLease&& other) noexcept;
        ~ReadLease();

        bool Valid() const;
        const CameraFrameHandle& Handle() const;
        port::LegacyCameraFrameView View() const;
        port::CameraPixelFrameView PixelView() const;

    private:
        friend class CameraFrameStore;
        ReadLease(CameraFrameStore& store, CameraFrameHandle handle);
        void Reset();

        CameraFrameStore* store_ = nullptr;
        CameraFrameHandle handle_{};
    };

    class WriteLease {
    public:
        WriteLease() = default;
        WriteLease(const WriteLease&) = delete;
        WriteLease& operator=(const WriteLease&) = delete;
        WriteLease(WriteLease&& other) noexcept;
        WriteLease& operator=(WriteLease&& other) noexcept;
        ~WriteLease();

        bool Valid() const;
        port::MutableLegacyCameraFrameView MutableView();
        port::MutableCameraPixelFrameView MutablePixelView();
        CameraFrameHandle Commit(uint64_t frame_id,
                                 uint64_t capture_time_ms,
                                 port::CameraRawFrameMetadata metadata,
                                 uint64_t submit_begin_us = 0);
        void Abort();

    private:
        friend class CameraFrameStore;
        WriteLease(CameraFrameStore& store,
                   CameraFrameHandle handle,
                   uint64_t reserve_begin_us);
        void ResetWithoutAbort();

        CameraFrameStore* store_ = nullptr;
        CameraFrameHandle handle_{};
        uint64_t reserve_begin_us_ = 0;
        bool active_ = false;
    };

    explicit CameraFrameStore(RuntimeState& state) : state_(state) {}

    CameraFrameHandle Submit(const port::LegacyCameraFrameView& view,
                             const port::CameraRawFrameMetadata& metadata);
    std::optional<WriteLease> ReserveWritable(int width,
                                              int height,
                                              port::CameraRawFrameMetadata metadata);
    std::optional<WriteLease> ReserveWritable(port::CameraFrameFormat format,
                                              int width,
                                              int height,
                                              int stride,
                                              port::CameraRawFrameMetadata metadata);
    std::optional<ReadLease> Acquire(const CameraFrameHandle& handle);
    std::optional<ReadLease> AcquireLatest();
    std::optional<ReadLease> AcquireLatestAfter(uint64_t last_seen_frame_id);
    std::optional<ReadLease> AcquireExact(uint64_t frame_id,
                                          uint64_t capture_time_ms);
    std::optional<CameraFrameHandle> TryGetLatestAfter(uint64_t last_seen_frame_id) const;
    CameraFrameHandle LatestHandle() const;
    std::optional<CameraFrameHandle> FindExact(uint64_t frame_id,
                                               uint64_t capture_time_ms) const;
    bool CopyFrame(const CameraFrameHandle& handle, port::LegacyCameraFrame& out) const;
    port::CameraFrameStoreHealth Health() const;

private:
    void ReleaseRead(const CameraFrameHandle& handle);
    void AbortWrite(const CameraFrameHandle& handle);

    RuntimeState& state_;
};

}  // namespace ls2k::runtime

#endif  // LS2K_RUNTIME_CAMERA_FRAME_STORE_HPP
