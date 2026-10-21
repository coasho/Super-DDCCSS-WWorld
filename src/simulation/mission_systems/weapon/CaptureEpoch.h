#pragma once

#include <cmath>
#include <optional>

namespace fsim::ms::weapon {
/// Maps actual pre-step/final sample ordinals to this World window. Native
/// timestamps remain untouched; a terminal no-op never advances this history.
class CaptureEpochV1 {
public:
    std::optional<double> capture(unsigned index,double dtS,bool terminal) noexcept {
        if (closed_ || !std::isfinite(dtS) || dtS<=0 ||
            (seen_ ? index!=lastIndex_+1 : index!=0)) return std::nullopt;
        const double offset=index*dtS;
        if (!std::isfinite(offset)) return std::nullopt;
        seen_=true; lastIndex_=index; lastOffsetS_=offset; closed_=terminal;
        return offset;
    }
    double lastOffsetS() const noexcept { return lastOffsetS_; }
private:
    bool seen_=false,closed_=false;
    unsigned lastIndex_=0;
    double lastOffsetS_=0;
};
}
