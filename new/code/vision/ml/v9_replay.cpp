#include "vision/ml/v9_replay.hpp"

#include <array>
#include <cstring>
#include <limits>

namespace ls2k::vision::ml {
namespace {

std::uint64_t LoadWord(const std::uint8_t* bytes) {
    std::uint64_t word = 0;
    std::memcpy(&word, bytes, sizeof(word));
    return word;
}

int Popcount64(std::uint64_t value) {
    value -= (value >> 1U) & UINT64_C(0x5555555555555555);
    value = (value & UINT64_C(0x3333333333333333)) +
            ((value >> 2U) & UINT64_C(0x3333333333333333));
    value = (value + (value >> 4U)) & UINT64_C(0x0f0f0f0f0f0f0f0f);
    return static_cast<int>((value * UINT64_C(0x0101010101010101)) >> 56U);
}

}  // namespace

bool ValidateV9Artifact(const port::V9ArtifactView& artifact) {
    if (artifact.template_codes == nullptr || artifact.template_parent == nullptr ||
        artifact.prototype_count == 0 || artifact.byte_count != port::kV9DescriptorBytes ||
        artifact.bit_count != port::kV9DescriptorBits) return false;
    for (std::size_t i = 0; i < artifact.prototype_count; ++i) {
        if (artifact.template_parent[i] > 2U) return false;
        const std::uint8_t last = artifact.template_codes[
            i * artifact.byte_count + artifact.byte_count - 1U];
        if ((last & 0xC0U) != 0U) return false;
    }
    return true;
}

port::V9ReplayResult ReplayV9Descriptor(const port::V9Descriptor& query,
                                        const port::V9ArtifactView& artifact) {
    port::V9ReplayResult out{};
    if (!query.valid || !ValidateV9Artifact(artifact)) return out;
    std::array<int, 3> best{std::numeric_limits<int>::max(),
                            std::numeric_limits<int>::max(),
                            std::numeric_limits<int>::max()};
    std::array<int, 3> index{-1, -1, -1};
    static_assert(port::kV9DescriptorBytes == 2U * sizeof(std::uint64_t),
                  "V9 replay word loop requires a 16-byte descriptor");
    const std::uint64_t query_low = LoadWord(query.bytes.data());
    const std::uint64_t query_high = LoadWord(query.bytes.data() + sizeof(std::uint64_t));
    for (std::size_t prototype = 0; prototype < artifact.prototype_count; ++prototype) {
        const int cls = artifact.template_parent[prototype];
        const std::uint8_t* code = artifact.template_codes + prototype * artifact.byte_count;
        const int distance = Popcount64(query_low ^ LoadWord(code)) +
                             Popcount64(query_high ^ LoadWord(code + sizeof(std::uint64_t)));
        if (distance < best[static_cast<std::size_t>(cls)]) {
            best[static_cast<std::size_t>(cls)] = distance;
            index[static_cast<std::size_t>(cls)] = static_cast<int>(prototype);
        }
    }
    int predicted = 0;
    if (best[1] < best[predicted]) predicted = 1;
    if (best[2] < best[predicted]) predicted = 2;
    int second = std::numeric_limits<int>::max();
    for (int cls = 0; cls < 3; ++cls)
        if (cls != predicted && best[static_cast<std::size_t>(cls)] < second)
            second = best[static_cast<std::size_t>(cls)];
    out.valid = index[static_cast<std::size_t>(predicted)] >= 0 && second != std::numeric_limits<int>::max();
    out.class_id = predicted;
    out.best_distance = best[static_cast<std::size_t>(predicted)];
    out.margin = out.valid ? second - out.best_distance : 0;
    out.prototype_index = index[static_cast<std::size_t>(predicted)];
    return out;
}

bool AcceptV9Result(const port::V9ReplayResult& result,
                    const port::MlV9Parameters& acceptance) {
    return result.valid && result.margin >= acceptance.min_margin &&
           result.best_distance <= acceptance.max_best_distance;
}

void ResetV9Acceptance(port::V9AcceptanceState& state) { state = {}; }

port::V9AcceptanceResult StepV9Acceptance(const port::V9ReplayResult& result,
                                          const port::MlV9Parameters& acceptance,
                                          port::V9AcceptanceState& state) {
    if (!AcceptV9Result(result, acceptance)) {
        ResetV9Acceptance(state);
        return {};
    }
    if (state.pending_class_id != result.class_id) {
        state.pending_class_id = result.class_id;
        state.consecutive_frames = 1;
    } else {
        ++state.consecutive_frames;
    }
    port::V9AcceptanceResult out{};
    out.accepted = state.consecutive_frames >= acceptance.confirm_frames;
    out.class_id = out.accepted ? result.class_id : -1;
    out.consecutive_frames = state.consecutive_frames;
    return out;
}

}  // namespace ls2k::vision::ml
