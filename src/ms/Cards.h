#pragma once

// Payload cards and fits (ADR-30, section 6). A card is one real payload type
// - its designation, its family, its parameters, each with a unit, a basis, an
// uncertainty and a public source (assets/payloads/<id>.json). A fit is what
// one aircraft type carries: its payloads, by card, with the source for each
// (aircraft/<type>/payloads.json), and the loadouts its stations take (MS-7).
// A store card is one store type - a missile, a bomb, a tank - as a payload
// card is (assets/stores/<id>.json).

#include "fsim/MissionSystems.h"

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace fsim::ms {

/// What a parameter's value rests on.
enum class Basis : std::uint8_t {
    Published, ///< stated in its source
    Derived,   ///< worked out from published values (its note says how)
    Assumed,   ///< not published: an engineering assumption (its note says why this value; its uncertainty is wide)
};

struct Source {
    std::string title, publisher, url, date;
};

struct Parameter {
    double value = 0.0;
    std::string unit;
    double uncertainty = 0.0; ///< one standard deviation, in `unit`
    Basis basis = Basis::Assumed;
    std::string source;       ///< a key of the card's sources (Published, Derived)
    std::string note;
};

/// One store station (a store-station card's, MS-7): where it is and what it takes.
struct Station {
    int number = 0;              ///< the aircraft's own numbering (the F-16's 1 to 9, left to right)
    std::string name;            ///< "left wingtip"
    double positionM[3] = {0, 0, 0}; ///< a store's CG on it, from the aircraft's CG: body axes, x forward, y right, z down
    std::string carriage;        ///< "rail": a missile fires off it under its own power; "ejector": the store is pushed clear;
                                 ///< "rail+ejector": a missile on its rail, any other store on its rack
    std::string bay;             ///< the weapon bay it is in - its name up to "bay" ("main bay" for "main bay 2"); empty: none
    std::string launcher;        ///< the rail or rack: "LAU-129", "BRU-61"
    /// The multiple rack it is a place on ("left pylon"): the stations that name the same one hang from one pylon and
    /// beam - one pylon's drag between them, one pylon drawn. Empty: a pylon of its own.
    std::string rack;
    double ejectionMs = 0.0;     ///< an ejector's push, down
    double extractionMs = 0.0;   ///< a cargo hold's: how fast the load leaves aft, over the ramp (drawn or rolling out)
    std::vector<std::string> takes; ///< the store classes it takes ("aam", "bomb", "tank", "pod")
    double maxKg = 0.0;          ///< the most it carries
    int capacity = 1;            ///< stores it holds (a multiple rack: more)
    Basis basis = Basis::Assumed; ///< of its place and what it takes
    std::string source, note;
};

struct Card {
    std::string id, designation, summary, maker;
    Family family = Family::AirRadar;
    std::map<std::string, Parameter> parameters;
    std::map<std::string, Source> sources;
    std::vector<Station> stations; ///< a store station card's (MS-7)
    /// A parameter's value, or `fallback` where the card has none.
    double value(const std::string& name, double fallback) const;
    const Station* station(int number) const noexcept;
};

/// One store type (MS-7; assets/stores/<id>.json): what a station carries.
struct StoreCard {
    std::string id, designation, summary, maker;
    std::string storeClass;      ///< "aam", "bomb", "tank", "pod": what stations take it
    std::string model;           ///< its flight model once released ("jsbsim:mk82"); empty: not built yet
    /// It may be released as a weapon (a strike). False: what it does after it leaves - a missile's powered flight - is
    /// not modelled; it loads and is weighed like any store, a strike with it is refused (StoreNotFlyable), and
    /// jettisoned it leaves as the inert body its model is.
    bool employable = true;
    bool powerable = false;      ///< powered before release (a missile's seeker and guidance)
    bool jettisonable = true;
    std::map<std::string, Parameter> parameters; ///< mass_kg, length_m, diameter_m, power_up_s ...
    std::map<std::string, Source> sources;
    /// A weapon's warhead (MS-7g): what it is ("WDU-41/B blast-fragmentation"), its kind ("fragmentation" or "rod";
    /// empty: none) and its explosive's name; its masses and the explosive's Gurney constant are parameters
    /// (warhead_kg, explosive_kg, gurney_kms)
    std::string warhead, warheadKind, explosive;
    double value(const std::string& name, double fallback) const;
};

/// A loadout (a fit's, MS-7): which stores its stations hold.
struct Loadout {
    std::string name, note, source;
    struct Item {
        std::string payload; ///< the station payload's name ("stations")
        int station = 0;
        std::string store;   ///< a store card's id
        int count = 1;
    };
    std::vector<Item> items;
};

struct FitEntry {
    std::string name;   ///< "radar", "station 3"
    std::string card;   ///< a card's id
    std::uint16_t count = 1;
    std::string source; ///< a key of the fit's sources
    std::string note;
};

struct Fit {
    std::string aircraft;
    std::string note;   ///< what it is, and what it leaves out (why, where it carries nothing)
    std::vector<FitEntry> payloads;
    std::map<std::string, Source> sources;
    /// The cards its payloads name, by id (loaded with it).
    std::map<std::string, std::shared_ptr<const Card>> cards;
    std::vector<Loadout> loadouts;   ///< (MS-7) the loads a scenario may give its stations (an aircraft starts empty)
    /// The store cards its loadouts name, and every one in the store library (a station may be loaded with any it takes).
    std::map<std::string, std::shared_ptr<const StoreCard>> stores;
    const Loadout* loadout(const std::string& name) const noexcept;
};

/// Read a store card. As a payload card: its parameters each with a value, a unit, an uncertainty and a basis.
std::shared_ptr<const StoreCard> readStoreCard(const std::filesystem::path& file, std::vector<std::string>& errors);
/// Every store card in `library` (share/flightsim/stores, or the source tree's assets/stores), by id.
std::map<std::string, std::shared_ptr<const StoreCard>> readStoreLibrary(const std::filesystem::path& library, std::vector<std::string>& errors);

/// Read a card. Every parameter needs a value, a unit, an uncertainty and a
/// basis; a Published or Derived one a source the card lists. Problems go to
/// `errors` (the card is still returned where it could be read).
std::shared_ptr<const Card> readCard(const std::filesystem::path& file, std::vector<std::string>& errors);

/// Read an aircraft's fit and the cards it names from `library`
/// (share/flightsim/payloads, or the source tree's assets/payloads), and the
/// store cards from `storeLibrary` (empty: <library>/../stores). Null where the
/// aircraft has no fit file; errors (an unknown card, a payload without a
/// source, a loadout a station cannot take) go to `errors`, and that payload or
/// loadout item is left out.
std::shared_ptr<const Fit> readFit(const std::filesystem::path& file, const std::filesystem::path& library, std::vector<std::string>& errors,
                                   const std::filesystem::path& storeLibrary = {});

/// The family's name in a card ("air_radar", ...) as a Family.
std::optional<Family> parseFamily(const std::string& name) noexcept;

/// The capabilities a payload of `family` gives, in order (the status capability first).
std::vector<CapabilityType> capabilitiesOf(Family family);
/// Whether a capability type is modelled in this build, and the ADR-30 stage that builds it where it is not.
control::Support capabilitySupport(CapabilityType type, std::uint8_t& stage) noexcept;

} // namespace fsim::ms
