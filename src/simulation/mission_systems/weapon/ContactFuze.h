#pragma once

#include "simulation/mission_systems/damage/Damage.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace fsim::ms::weapon {

/// A captured nose-tip position in the target's body axes, relative to its
/// fixed model origin (metres). Offsets belong to one World decision step.
/// Each adjacent pair defines one linear segment; no motion is integrated here.
struct ContactSample {
    double offsetS = 0;
    damage::V3 pointBody;
    bool available = true;
};

struct ContactWindow {
    double ageBeforeS = 0;
    double durationS = 0;
    double armingS = 0;
    double lifeS = 0;
    double cutoffS = 0;
    bool exclusiveCutoff = false; ///< Native arrest/divergence excludes its endpoint.
};

enum class ContactKind { Panel, Component };

struct ContactFuzeResult {
    bool valid = false;
    bool contact = false;        ///< First physical crossing, including an unarmed one.
    bool triggered = false;      ///< That crossing is armed and live.
    bool initialOverlap = false; ///< No new crossing can be inferred in this sweep.
    double offsetS = std::numeric_limits<double>::quiet_NaN();
    double fraction = std::numeric_limits<double>::quiet_NaN(); ///< Original decision-step fraction.
    ContactKind kind = ContactKind::Panel;
    std::size_t index = std::numeric_limits<std::size_t>::max(); ///< Immutable Airframe source index.
    /// Exact selected-face contact, in target body metres from fixed origin.
    damage::V3 pointBody{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()};
    /// Unit selected-face normal, oriented against the incoming captured
    /// segment. Coplanar contact has zero normal closing movement. Unavailable
    /// point/normal values remain NaN when no contact is returned.
    damage::V3 normalBody{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN()};
};

/// Sweep a point against finite panel faces and validated authored closed
/// contact meshes. Damage boxes are not physical narrow-phase geometry;
/// legacy data without contact_geometry v1 is unavailable. No radius, skin thickness, closest-approach substitution or arming
/// restart is added. Life is exclusive; arming is inclusive. The first physical
/// crossing is selected before testing arming. Initial overlap (including a
/// boundary) suppresses this target's whole sweep; an unavailable sample rejects
/// it rather than bridging a gap. A short history is never extrapolated.
///
/// Pure per-target evaluation: compare all candidates before committing the
/// globally earliest crossing to ContactFuzeState. Ownership caps are hidden
/// while their adjacent owner is attached. Exact ties prefer authored skin,
/// then a panel, then its lower immutable source index. The caller resolves target-ID ties.
ContactFuzeResult contactFuze(const std::vector<ContactSample>& path,
                              const damage::AttachedGeometryView& geometry,
                              const ContactWindow& window);

/// One release's contact-initiation channel only. In particular, an unarmed
/// first crossing cannot become an armed exit/re-entry on a later decision step.
/// This does not end a weapon or alter its separate proximity-fuze channel.
class ContactFuzeState {
public:
    bool consumed() const noexcept { return consumed_; }
    void reset() noexcept { consumed_ = false; }
    /// Commit only the selected earliest candidate; true authorizes initiation
    /// through the existing detonation path. Invalid/overlap/no-contact results
    /// have no effect. A valid unarmed contact consumes the channel too.
    bool commit(const ContactFuzeResult& crossing) noexcept;

private:
    bool consumed_ = false;
};

} // namespace fsim::ms::weapon
