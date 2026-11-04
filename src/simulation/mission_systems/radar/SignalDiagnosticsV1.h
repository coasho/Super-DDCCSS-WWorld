#pragma once
#include "Signal.h"

namespace fsim::ms::radar::signal {
/// Additive private diagnostic sink; existing Snapshot/dwell layouts and
/// signatures remain unchanged. All powers use Snapshot's mean thermal-noise
/// per filter normalization. Untested/notched cells have tested=0; their zero
/// array slots are unavailable, not physical zero detection thresholds.
struct CellDiagnosticsV1 {
    bool kept=false;
    int interval=0,rows=0,filters=0;
    std::vector<std::uint8_t> tested;
    std::vector<float> thermalNoiseRelative,localNoiseRelative;
    std::vector<float> cfarThresholdRelative,clutterThresholdRelative,decisionThresholdRelative;
};
/// Actual folded native-cell detection before M-of-N. Epochs and energy
/// fractions are supplied by a finite physical listener, not source truth.
struct ObservedPeakV1 {
    unsigned interval=0;
    double timeS=0,rangeM=0,rateMs=0,snr=0,monopulseAz=0,monopulseEl=0;
    std::uint32_t truth=0; ///< Post-detection referee scoring only.
};
struct DwellObservationV1 {
    std::vector<double> intervalEnergyFraction,intervalTimeS;
    std::vector<ObservedPeakV1> peaks;
};
std::vector<Hit> dwellWithDiagnosticsV1(const Waveform&,const std::vector<Echo>&,const Settings&,Stream&,
    Stats*,const ClutterMap*,Snapshot*,CellDiagnosticsV1*);
/// Processes actual observed intervals and exports their measured peaks;
/// coincidence is performed separately across real observation epochs.
/// Existing dwell/null paths retain their arithmetic and RNG sequence.
bool dwellObservedV1(const Waveform&,const std::vector<Echo>&,const Settings&,Stream&,
    Stats*,const ClutterMap*,Snapshot*,CellDiagnosticsV1*,DwellObservationV1&);
} // namespace fsim::ms::radar::signal
