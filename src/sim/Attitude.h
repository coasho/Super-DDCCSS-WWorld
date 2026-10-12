#pragma once

#include "core/Attitude.h"

// Preserve the existing internal sim interface; the pure math belongs to core.
namespace fsim::sim::attitude {
using core::attitude::bodyToEcefFromQuaternion;
using core::attitude::slerp;
} // namespace fsim::sim::attitude
