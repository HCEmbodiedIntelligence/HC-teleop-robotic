#include "hc_control/runtime.hpp"

namespace hc {
// Keep initialization of the large fixed-capacity queues out of optimized
// application call sites. This runs once, before the periodic thread starts.
Shared::Shared() = default;
} // namespace hc
