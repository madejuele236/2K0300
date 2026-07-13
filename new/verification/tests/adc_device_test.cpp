#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <type_traits>
#include <unistd.h>

#include "platform/true_ls2k0300/adc_device.hpp"

namespace {

using ls2k::platform::true_ls2k0300::AdcDevice;
using ls2k::platform::true_ls2k0300::AdcReadError;
using ls2k::platform::true_ls2k0300::AdcSampleResult;

void Expect(bool condition) {
    if (!condition) {
        std::abort();
    }
}

class TempFile final {
public:
    explicit TempFile(const char* contents) {
        char pattern[] = "/tmp/ls2k-adc-device-XXXXXX";
        const int fd = mkstemp(pattern);
        Expect(fd >= 0);
        path_ = pattern;
        const std::size_t size = std::char_traits<char>::length(contents);
        Expect(write(fd, contents, size) == static_cast<ssize_t>(size));
        Expect(close(fd) == 0);
    }

    ~TempFile() { unlink(path_.c_str()); }

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

void TestContractAndSuccess() {
    static_assert(std::is_trivially_copyable_v<AdcSampleResult>);
    static_assert(!std::is_copy_constructible_v<AdcDevice>);
    static_assert(!std::is_copy_assignable_v<AdcDevice>);
    static_assert(std::is_nothrow_move_constructible_v<AdcDevice>);

    TempFile input("  731\n");
    AdcDevice adc(input.path());
    const AdcSampleResult result = adc.ReadRaw();
    Expect(result.ok());
    Expect(result.raw_value == 731);
    Expect(result.system_errno == 0);
    Expect(adc.path() == input.path());
}

void TestMissingPath() {
    AdcDevice adc("/tmp/ls2k-adc-device-does-not-exist");
    const AdcSampleResult result = adc.ReadRaw();
    Expect(!result.ok());
    Expect(result.error == AdcReadError::kOpenFailed);
    Expect(result.system_errno != 0);
}

void TestParseFailures() {
    TempFile malformed("12x\n");
    AdcDevice malformed_adc(malformed.path());
    Expect(malformed_adc.ReadRaw().error == AdcReadError::kParseFailed);

    TempFile overflow("999999999999999999999999\n");
    AdcDevice overflow_adc(overflow.path());
    Expect(overflow_adc.ReadRaw().error == AdcReadError::kOutOfRange);

    TempFile empty(" \n\t");
    AdcDevice empty_adc(empty.path());
    Expect(empty_adc.ReadRaw().error == AdcReadError::kEmpty);
}

}  // namespace

int main() {
    TestContractAndSuccess();
    TestMissingPath();
    TestParseFailures();
    return EXIT_SUCCESS;
}
