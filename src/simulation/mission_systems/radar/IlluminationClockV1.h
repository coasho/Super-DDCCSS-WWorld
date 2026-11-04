#pragma once

namespace fsim::ms::radar {
/// Additive private view of the native transmitter scheduler, not a new
/// transmitter. epochS is the current native dwell's start; the PRF intervals
/// run in their declared order, at the declared pulse times, once per periodS.
/// The listener itself bounds validity by when it first observes this program.
struct IlluminationClockV1 {
    bool illuminating=false;
    double epochS=0,periodS=0;
};
} // namespace fsim::ms::radar
