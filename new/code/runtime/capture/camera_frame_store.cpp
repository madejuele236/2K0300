#include "runtime/capture/camera_frame_store.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

namespace ls2k::runtime {
namespace {

uint64_t NowUs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

}  // namespace

CameraFrameStore::ReadLease::ReadLease(CameraFrameStore& store, CameraFrameHandle handle)
    : store_(&store), handle_(std::move(handle)) {}

CameraFrameStore::ReadLease::ReadLease(ReadLease&& other) noexcept
    : store_(other.store_), handle_(std::move(other.handle_)) {
    other.store_ = nullptr;
    other.handle_ = {};
}

CameraFrameStore::ReadLease& CameraFrameStore::ReadLease::operator=(ReadLease&& other) noexcept {
    if (this != &other) {
        Reset();
        store_ = other.store_;
        handle_ = std::move(other.handle_);
        other.store_ = nullptr;
        other.handle_ = {};
    }
    return *this;
}

CameraFrameStore::ReadLease::~ReadLease() {
    Reset();
}

bool CameraFrameStore::ReadLease::Valid() const {
    return store_ != nullptr && handle_.valid;
}

const CameraFrameHandle& CameraFrameStore::ReadLease::Handle() const {
    return handle_;
}

port::LegacyCameraFrameView CameraFrameStore::ReadLease::View() const {
    port::LegacyCameraFrameView view{};
    if (!Valid() || handle_.slot_id >= store_->state_.camera_frame_slots.size()) {
        return view;
    }
    const OwnedCameraFrameSlot& slot = store_->state_.camera_frame_slots[handle_.slot_id];
    if (slot.format != port::CameraFrameFormat::kGray) {
        return view;
    }
    view.gray = slot.gray.data();
    view.width = slot.width;
    view.height = slot.height;
    view.stride = slot.stride;
    view.frame_id = slot.frame_id;
    view.capture_time_ms = slot.capture_time_ms;
    return view;
}

port::CameraPixelFrameView CameraFrameStore::ReadLease::PixelView() const {
    port::CameraPixelFrameView view{};
    if (!Valid() || handle_.slot_id >= store_->state_.camera_frame_slots.size()) {
        return view;
    }
    const OwnedCameraFrameSlot& slot = store_->state_.camera_frame_slots[handle_.slot_id];
    view.valid = CameraFrameSlotReadyForHandle(slot, handle_);
    view.format = slot.format;
    view.data = slot.gray.data();
    view.width = slot.width;
    view.height = slot.height;
    view.stride = slot.stride;
    view.frame_id = slot.frame_id;
    view.capture_time_ms = slot.capture_time_ms;
    return view;
}

void CameraFrameStore::ReadLease::Reset() {
    if (store_ != nullptr && handle_.valid) {
        store_->ReleaseRead(handle_);
    }
    store_ = nullptr;
    handle_ = {};
}

CameraFrameStore::WriteLease::WriteLease(CameraFrameStore& store,
                                         CameraFrameHandle handle,
                                         uint64_t reserve_begin_us)
    : store_(&store), handle_(std::move(handle)), reserve_begin_us_(reserve_begin_us), active_(true) {}

CameraFrameStore::WriteLease::WriteLease(WriteLease&& other) noexcept
    : store_(other.store_),
      handle_(std::move(other.handle_)),
      reserve_begin_us_(other.reserve_begin_us_),
      active_(other.active_) {
    other.ResetWithoutAbort();
}

CameraFrameStore::WriteLease& CameraFrameStore::WriteLease::operator=(WriteLease&& other) noexcept {
    if (this != &other) {
        Abort();
        store_ = other.store_;
        handle_ = std::move(other.handle_);
        reserve_begin_us_ = other.reserve_begin_us_;
        active_ = other.active_;
        other.ResetWithoutAbort();
    }
    return *this;
}

CameraFrameStore::WriteLease::~WriteLease() {
    Abort();
}

bool CameraFrameStore::WriteLease::Valid() const {
    return store_ != nullptr && active_ && handle_.valid;
}

port::MutableLegacyCameraFrameView CameraFrameStore::WriteLease::MutableView() {
    port::MutableLegacyCameraFrameView view{};
    if (!Valid() || handle_.slot_id >= store_->state_.camera_frame_slots.size()) {
        return view;
    }
    OwnedCameraFrameSlot& slot = store_->state_.camera_frame_slots[handle_.slot_id];
    if (slot.format != port::CameraFrameFormat::kGray) {
        return view;
    }
    view.gray = slot.gray.data();
    view.width = slot.width;
    view.height = slot.height;
    view.stride = slot.stride;
    view.frame_id = slot.frame_id;
    view.capture_time_ms = slot.capture_time_ms;
    return view;
}

port::MutableCameraPixelFrameView CameraFrameStore::WriteLease::MutablePixelView() {
    port::MutableCameraPixelFrameView view{};
    if (!Valid() || handle_.slot_id >= store_->state_.camera_frame_slots.size()) {
        return view;
    }
    OwnedCameraFrameSlot& slot = store_->state_.camera_frame_slots[handle_.slot_id];
    view.valid = true;
    view.format = slot.format;
    view.data = slot.gray.data();
    view.width = slot.width;
    view.height = slot.height;
    view.stride = slot.stride;
    view.frame_id = slot.frame_id;
    view.capture_time_ms = slot.capture_time_ms;
    return view;
}

CameraFrameHandle CameraFrameStore::WriteLease::Commit(uint64_t frame_id,
                                                       uint64_t capture_time_ms,
                                                       port::CameraRawFrameMetadata metadata,
                                                       uint64_t submit_begin_us) {
    CameraFrameHandle published{};
    if (!Valid()) {
        return published;
    }
    const uint64_t timing_begin_us = submit_begin_us == 0 ? reserve_begin_us_ : submit_begin_us;
    std::lock_guard<std::mutex> lock(store_->state_.shared_mutex);
    if (handle_.slot_id >= store_->state_.camera_frame_slots.size()) {
        ResetWithoutAbort();
        return published;
    }
    OwnedCameraFrameSlot& slot = store_->state_.camera_frame_slots[handle_.slot_id];
    if (slot.state != OwnedCameraFrameSlotState::kEncoding ||
        slot.generation != handle_.generation) {
        ++store_->state_.camera_frame_store_health.dropped_frame_count;
        ResetWithoutAbort();
        return published;
    }

    slot.frame_id = frame_id;
    slot.capture_time_ms = capture_time_ms;
    slot.metadata = metadata;
    slot.metadata.frame_id = frame_id;
    slot.metadata.capture_time_ms = capture_time_ms;
    slot.metadata.store_submit_us = NowUs() - timing_begin_us;
    slot.state = OwnedCameraFrameSlotState::kReady;

    published.valid = true;
    published.slot_id = slot.slot_id;
    published.generation = slot.generation;
    published.frame_id = slot.frame_id;
    published.capture_time_ms = slot.capture_time_ms;
    published.format = slot.format;
    published.width = slot.width;
    published.height = slot.height;
    published.stride = slot.stride;
    published.metadata = slot.metadata;
    store_->state_.latest_camera_frame = published;
    store_->state_.recent_camera_captures.Push(published);
    ++store_->state_.camera_frame_store_health.submitted_frame_count;
    ResetWithoutAbort();
    return published;
}

void CameraFrameStore::WriteLease::Abort() {
    if (store_ != nullptr && active_ && handle_.valid) {
        store_->AbortWrite(handle_);
    }
    ResetWithoutAbort();
}

void CameraFrameStore::WriteLease::ResetWithoutAbort() {
    store_ = nullptr;
    handle_ = {};
    reserve_begin_us_ = 0;
    active_ = false;
}

CameraFrameHandle CameraFrameStore::Submit(const port::LegacyCameraFrameView& view,
                                           const port::CameraRawFrameMetadata& metadata) {
    const uint64_t submit_begin_us = NowUs();
    CameraFrameHandle handle{};
    if (!view.Valid() ||
        view.width > port::kCompiledCameraFrameWidth ||
        view.height > port::kCompiledCameraFrameHeight) {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        ++state_.camera_frame_store_health.dropped_frame_count;
        return handle;
    }

    std::optional<WriteLease> lease = ReserveWritable(view.width, view.height, metadata);
    if (!lease.has_value()) {
        return handle;
    }
    port::MutableLegacyCameraFrameView dst_view = lease->MutableView();
    if (!dst_view.Valid()) {
        lease->Abort();
        return handle;
    }
    for (int row = 0; row < view.height; ++row) {
        const std::uint8_t* src =
            view.gray + static_cast<std::size_t>(row) * static_cast<std::size_t>(view.stride);
        std::uint8_t* dst =
            dst_view.gray + static_cast<std::size_t>(row) * static_cast<std::size_t>(dst_view.stride);
        std::copy(src, src + view.width, dst);
    }
    return lease->Commit(view.frame_id, view.capture_time_ms, metadata, submit_begin_us);
}

std::optional<CameraFrameStore::WriteLease> CameraFrameStore::ReserveWritable(
    int width,
    int height,
    port::CameraRawFrameMetadata metadata) {
    return ReserveWritable(port::CameraFrameFormat::kGray, width, height, width, metadata);
}

std::optional<CameraFrameStore::WriteLease> CameraFrameStore::ReserveWritable(
    port::CameraFrameFormat format,
    int width,
    int height,
    int stride,
    port::CameraRawFrameMetadata metadata) {
    const uint64_t reserve_begin_us = NowUs();
    if (width <= 0 ||
        height <= 0 ||
        stride < port::MinimumStrideBytes(format, width) ||
        width > port::kCompiledCameraFrameWidth ||
        height > port::kCompiledCameraFrameHeight ||
        static_cast<std::size_t>(stride) * static_cast<std::size_t>(height) >
            port::kCompiledCameraFrameMaxBytes) {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        ++state_.camera_frame_store_health.dropped_frame_count;
        return std::nullopt;
    }

    CameraFrameHandle handle{};
    std::size_t selected = 0;
    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        constexpr std::size_t kSlotCount = 3;
        selected = kSlotCount;
        for (std::size_t attempt = 0; attempt < kSlotCount; ++attempt) {
            const std::size_t index = (state_.next_camera_frame_slot + attempt) % kSlotCount;
            if (state_.camera_frame_slots[index].state != OwnedCameraFrameSlotState::kEncoding &&
                state_.camera_frame_slot_readers[index] == 0U) {
                selected = index;
                break;
            }
        }
        if (selected == kSlotCount) {
            ++state_.camera_frame_store_health.dropped_frame_count;
            return std::nullopt;
        }
        state_.next_camera_frame_slot = (selected + 1) % kSlotCount;

        OwnedCameraFrameSlot& slot = state_.camera_frame_slots[selected];
        if (slot.state == OwnedCameraFrameSlotState::kReady) {
            ++state_.camera_frame_store_health.overwritten_frame_count;
        }
        slot.slot_id = selected;
        slot.state = OwnedCameraFrameSlotState::kEncoding;
        slot.generation = slot.generation == UINT64_MAX ? 1 : slot.generation + 1;
        slot.frame_id = 0;
        slot.capture_time_ms = 0;
        slot.format = format;
        slot.width = width;
        slot.height = height;
        slot.stride = stride;
        slot.metadata = metadata;
        handle.valid = true;
        handle.slot_id = selected;
        handle.generation = slot.generation;
        handle.format = slot.format;
        handle.width = slot.width;
        handle.height = slot.height;
        handle.stride = slot.stride;
        handle.metadata = slot.metadata;
    }
    return WriteLease(*this, handle, reserve_begin_us);
}

std::optional<CameraFrameStore::ReadLease> CameraFrameStore::Acquire(
    const CameraFrameHandle& handle) {
    if (!handle.valid || handle.slot_id >= state_.camera_frame_slots.size()) {
        return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    const OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
    if (!CameraFrameSlotReadyForHandle(slot, handle)) {
        return std::nullopt;
    }
    ++state_.camera_frame_slot_readers[handle.slot_id];
    return ReadLease(*this, handle);
}

std::optional<CameraFrameStore::ReadLease> CameraFrameStore::AcquireLatest() {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    const CameraFrameHandle handle = state_.latest_camera_frame;
    if (!handle.valid || handle.slot_id >= state_.camera_frame_slots.size()) {
        return std::nullopt;
    }
    const OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
    if (!CameraFrameSlotReadyForHandle(slot, handle)) {
        return std::nullopt;
    }
    ++state_.camera_frame_slot_readers[handle.slot_id];
    return ReadLease(*this, handle);
}

std::optional<CameraFrameStore::ReadLease> CameraFrameStore::AcquireLatestAfter(
    uint64_t last_seen_frame_id) {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    const CameraFrameHandle handle = state_.latest_camera_frame;
    if (!handle.valid ||
        handle.frame_id == last_seen_frame_id ||
        handle.slot_id >= state_.camera_frame_slots.size()) {
        return std::nullopt;
    }
    const OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
    if (!CameraFrameSlotReadyForHandle(slot, handle)) {
        return std::nullopt;
    }
    ++state_.camera_frame_slot_readers[handle.slot_id];
    return ReadLease(*this, handle);
}

std::optional<CameraFrameStore::ReadLease> CameraFrameStore::AcquireExact(
    uint64_t frame_id,
    uint64_t capture_time_ms) {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    const CameraFrameHandle* matched =
        state_.recent_camera_captures.FindExact(frame_id, capture_time_ms);
    if (matched == nullptr) {
        ++state_.camera_frame_store_health.lookup_miss_count;
        return std::nullopt;
    }
    const CameraFrameHandle handle = *matched;
    if (!handle.valid || handle.slot_id >= state_.camera_frame_slots.size()) {
        return std::nullopt;
    }
    const OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
    if (!CameraFrameSlotReadyForHandle(slot, handle)) {
        return std::nullopt;
    }
    ++state_.camera_frame_slot_readers[handle.slot_id];
    return ReadLease(*this, handle);
}

std::optional<CameraFrameHandle> CameraFrameStore::TryGetLatestAfter(
    uint64_t last_seen_frame_id) const {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    if (!state_.latest_camera_frame.valid ||
        state_.latest_camera_frame.frame_id == last_seen_frame_id) {
        return std::nullopt;
    }
    return state_.latest_camera_frame;
}

CameraFrameHandle CameraFrameStore::LatestHandle() const {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    return state_.latest_camera_frame;
}

std::optional<CameraFrameHandle> CameraFrameStore::FindExact(uint64_t frame_id,
                                                             uint64_t capture_time_ms) const {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    const CameraFrameHandle* handle =
        state_.recent_camera_captures.FindExact(frame_id, capture_time_ms);
    if (handle == nullptr) {
        ++state_.camera_frame_store_health.lookup_miss_count;
        return std::nullopt;
    }
    return *handle;
}

bool CameraFrameStore::CopyFrame(const CameraFrameHandle& handle,
                                 port::LegacyCameraFrame& out) const {
    if (!handle.valid || handle.slot_id >= state_.camera_frame_slots.size()) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        const OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
        if (!CameraFrameSlotReadyForHandle(slot, handle)) {
            return false;
        }
        ++state_.camera_frame_slot_readers[handle.slot_id];
    }

    const OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
    CopyOwnedCameraFramePixels(slot, out);

    {
        std::lock_guard<std::mutex> lock(state_.shared_mutex);
        if (state_.camera_frame_slot_readers[handle.slot_id] > 0U) {
            --state_.camera_frame_slot_readers[handle.slot_id];
        }
    }
    return true;
}

port::CameraFrameStoreHealth CameraFrameStore::Health() const {
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    return state_.camera_frame_store_health;
}

void CameraFrameStore::ReleaseRead(const CameraFrameHandle& handle) {
    if (!handle.valid || handle.slot_id >= state_.camera_frame_slot_readers.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    if (state_.camera_frame_slot_readers[handle.slot_id] > 0U) {
        --state_.camera_frame_slot_readers[handle.slot_id];
    }
}

void CameraFrameStore::AbortWrite(const CameraFrameHandle& handle) {
    if (!handle.valid || handle.slot_id >= state_.camera_frame_slots.size()) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    OwnedCameraFrameSlot& slot = state_.camera_frame_slots[handle.slot_id];
    if (slot.state == OwnedCameraFrameSlotState::kEncoding &&
        slot.generation == handle.generation) {
        slot.state = OwnedCameraFrameSlotState::kFree;
    }
}

}  // namespace ls2k::runtime
