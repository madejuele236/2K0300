#ifndef PRIMER_CODE_RUNTIME_APPLICATION_HPP_
#define PRIMER_CODE_RUNTIME_APPLICATION_HPP_

// Compatibility callback retained for the PIT driver and static comparison
// with the original application entry point.
void pit_callback();

namespace primer::runtime {

int RunApplication();

}  // namespace primer::runtime

#endif  // PRIMER_CODE_RUNTIME_APPLICATION_HPP_
