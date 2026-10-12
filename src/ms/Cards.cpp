#include "ms/Cards.h"

#include "core/Json.h"

#include <algorithm>
#include <fstream>
#include <sstream>

namespace fsim::ms {

namespace {

std::optional<core::Json> readJson(const std::filesystem::path& file, std::vector<std::string>& errors) {
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::nullopt;
    std::stringstream ss;
    ss << in.rdbuf();
    try {
        return core::Json::parse(ss.str(), file.string());
    } catch (const std::exception& ex) {
        errors.push_back(ex.what());
        return std::nullopt;
    }
}

std::map<std::string, Source> readSources(const core::Json& doc) {
    std::map<std::string, Source> out;
    if (const core::Json* s = doc.find("sources"); s && s->isObject())
        for (const auto& [key, value] : s->asObject())
            out[key] = Source{value.string("title", ""), value.string("publisher", ""), value.string("url", ""), value.string("date", "")};
    return out;
}

/// A card's parameters: each a value, a unit, an uncertainty and a basis; a published or derived one a source the card
/// lists, a derived or assumed one a note.
std::map<std::string, Parameter> readParameters(const core::Json& doc, const std::map<std::string, Source>& sources, const std::string& where,
                                                std::vector<std::string>& errors) {
    std::map<std::string, Parameter> parameters;
    if (const core::Json* params = doc.find("parameters"); params && params->isObject())
        for (const auto& [name, p] : params->asObject()) {
            Parameter out;
            const std::string at = where + "parameter '" + name + "': ";
            if (!p.has("value") || !p.has("unit") || !p.has("uncertainty") || !p.has("basis")) {
                errors.push_back(at + "needs a value, a unit, an uncertainty and a basis");
                continue;
            }
            out.value = p.number("value", 0.0);
            out.unit = p.string("unit", "");
            out.uncertainty = p.number("uncertainty", 0.0);
            const std::string basis = p.string("basis", "");
            out.basis = basis == "published" ? Basis::Published : basis == "derived" ? Basis::Derived : Basis::Assumed;
            if (basis != "published" && basis != "derived" && basis != "assumed") errors.push_back(at + "unknown basis '" + basis + "'");
            out.source = p.string("source", "");
            out.note = p.string("note", "");
            if (out.basis != Basis::Assumed && !sources.count(out.source)) errors.push_back(at + "a " + basis + " value needs a source the card lists");
            if (out.basis != Basis::Published && out.note.empty()) errors.push_back(at + "a " + basis + " value needs a note saying how or why");
            if (!(out.uncertainty >= 0.0)) errors.push_back(at + "its uncertainty must be 0 or more");
            parameters[name] = out;
        }
    return parameters;
}

Basis parseBasis(const std::string& basis) noexcept {
    return basis == "published" ? Basis::Published : basis == "derived" ? Basis::Derived : Basis::Assumed;
}

/// A store-station card's stations.
std::vector<Station> readStations(const core::Json& doc, const std::map<std::string, Source>& sources, const std::string& where,
                                  std::vector<std::string>& errors) {
    std::vector<Station> out;
    const core::Json* list = doc.find("stations");
    if (!list || !list->isArray()) return out;
    for (const auto& s : list->asArray()) {
        Station st;
        st.number = static_cast<int>(s.number("number", 0.0));
        st.name = s.string("name", "");
        const std::string at = where + "station " + std::to_string(st.number) + ": ";
        if (const core::Json* p = s.find("position_m"); p && p->isArray() && p->asArray().size() == 3)
            for (int i = 0; i < 3; ++i) st.positionM[i] = p->asArray()[static_cast<std::size_t>(i)].asNumber();
        else
            errors.push_back(at + "needs its position_m [x forward, y right, z down] from the CG");
        st.carriage = s.string("carriage", "");
        if (st.carriage != "rail" && st.carriage != "ejector" && st.carriage != "rail+ejector")
            errors.push_back(at + "its carriage is 'rail', 'ejector' or 'rail+ejector'");
        // its weapon bay, by its name: "main bay 2", "forward main bay, left", "left side bay" - the stations whose names
        // agree up to "bay" share its doors
        if (const auto b = st.name.find("bay"); b != std::string::npos) st.bay = st.name.substr(0, b + 3);
        st.launcher = s.string("launcher", "");
        st.rack = s.string("rack", "");
        st.ejectionMs = s.number("ejection_ms", st.carriage.find("ejector") != std::string::npos ? 3.0 : 0.0);
        st.extractionMs = s.number("extraction_ms", 0.0);
        if (st.extractionMs < 0.0) errors.push_back(at + "its extraction_ms is how fast its load leaves aft: not negative");
        if (const core::Json* t = s.find("takes"); t && t->isArray())
            for (const auto& c : t->asArray()) st.takes.push_back(c.asString());
        if (st.takes.empty()) errors.push_back(at + "takes no store class");
        st.maxKg = s.number("max_kg", 0.0);
        if (!(st.maxKg > 0.0)) errors.push_back(at + "needs its max_kg");
        st.capacity = std::max(1, static_cast<int>(s.number("capacity", 1.0)));
        const std::string basis = s.string("basis", "");
        if (basis != "published" && basis != "derived" && basis != "assumed") errors.push_back(at + "unknown basis '" + basis + "'");
        st.basis = parseBasis(basis);
        st.source = s.string("source", "");
        st.note = s.string("note", "");
        if (st.basis != Basis::Assumed && !sources.count(st.source)) errors.push_back(at + "a " + basis + " station needs a source the card lists");
        if (st.basis != Basis::Published && st.note.empty()) errors.push_back(at + "a " + basis + " station needs a note saying how or why");
        for (const auto& other : out)
            if (other.number == st.number) errors.push_back(at + "a second station of that number");
        out.push_back(std::move(st));
    }
    return out;
}

} // namespace

double Card::value(const std::string& name, double fallback) const {
    const auto it = parameters.find(name);
    return it == parameters.end() ? fallback : it->second.value;
}

const Station* Card::station(int number) const noexcept {
    for (const auto& s : stations)
        if (s.number == number) return &s;
    return nullptr;
}

double StoreCard::value(const std::string& name, double fallback) const {
    const auto it = parameters.find(name);
    return it == parameters.end() ? fallback : it->second.value;
}

const Loadout* Fit::loadout(const std::string& name) const noexcept {
    for (const auto& l : loadouts)
        if (l.name == name) return &l;
    return nullptr;
}

std::shared_ptr<const StoreCard> readStoreCard(const std::filesystem::path& file, std::vector<std::string>& errors) {
    const auto doc = readJson(file, errors);
    if (!doc) {
        errors.push_back(file.string() + ": cannot be read");
        return nullptr;
    }
    const std::string where = file.string() + ": ";
    try {
        auto card = std::make_shared<StoreCard>();
        card->id = doc->string("id", "");
        card->designation = doc->string("designation", "");
        card->summary = doc->string("summary", "");
        card->maker = doc->string("maker", "");
        if (card->id.empty() || card->designation.empty()) errors.push_back(where + "a store card needs an id and a designation");
        if (card->id != file.stem().string()) errors.push_back(where + "its id '" + card->id + "' is not its file's name");
        card->storeClass = doc->string("class", "");
        if (card->storeClass.empty()) errors.push_back(where + "a store card needs its class");
        card->model = doc->string("model", "");
        card->seeker = doc->string("seeker", "");
        card->powerable = doc->boolean("powerable", false);
        card->jettisonable = doc->boolean("jettisonable", true);
        card->sources = readSources(*doc);
        if (card->sources.empty()) errors.push_back(where + "a store card needs at least one source");
        card->parameters = readParameters(*doc, card->sources, where, errors);
        if (!card->parameters.count("mass_kg")) errors.push_back(where + "a store card needs its mass_kg");
        card->warhead = doc->string("warhead", "");
        card->warheadKind = doc->string("warhead_kind", "");
        card->explosive = doc->string("explosive", "");
        if (!card->warheadKind.empty() && card->warheadKind != "fragmentation" && card->warheadKind != "rod")
            errors.push_back(where + "warhead_kind is 'fragmentation' or 'rod'");
        if (!card->warheadKind.empty() && (!card->parameters.count("warhead_kg") || !card->parameters.count("explosive_kg")))
            errors.push_back(where + "a warhead needs its warhead_kg and explosive_kg");
        return card;
    } catch (const std::exception& ex) {
        errors.push_back(where + ex.what());
        return nullptr;
    }
}

std::map<std::string, std::shared_ptr<const StoreCard>> readStoreLibrary(const std::filesystem::path& library, std::vector<std::string>& errors) {
    std::map<std::string, std::shared_ptr<const StoreCard>> out;
    std::error_code ec;
    if (!std::filesystem::is_directory(library, ec)) return out;
    for (const auto& f : std::filesystem::directory_iterator(library, ec))
        if (f.path().extension() == ".json")
            if (auto card = readStoreCard(f.path(), errors)) out[card->id] = std::move(card);
    return out;
}

std::optional<Family> parseFamily(const std::string& name) noexcept {
    for (std::size_t i = 0; i < static_cast<std::size_t>(Family::Count); ++i)
        if (name == familyName(static_cast<Family>(i))) return static_cast<Family>(i);
    return std::nullopt;
}

std::shared_ptr<const Card> readCard(const std::filesystem::path& file, std::vector<std::string>& errors) {
    const auto doc = readJson(file, errors);
    if (!doc) {
        errors.push_back(file.string() + ": cannot be read");
        return nullptr;
    }
    const std::string where = file.string() + ": ";
    try {
        auto card = std::make_shared<Card>();
        card->id = doc->string("id", "");
        card->designation = doc->string("designation", "");
        card->summary = doc->string("summary", "");
        card->maker = doc->string("maker", "");
        if (card->id.empty() || card->designation.empty()) errors.push_back(where + "a card needs an id and a designation");
        if (card->id != file.stem().string()) errors.push_back(where + "its id '" + card->id + "' is not its file's name");
        const auto family = parseFamily(doc->string("family", ""));
        if (!family) errors.push_back(where + "unknown family '" + doc->string("family", "") + "'");
        else card->family = *family;
        card->sources = readSources(*doc);
        if (card->sources.empty()) errors.push_back(where + "a card needs at least one source");
        card->parameters = readParameters(*doc, card->sources, where, errors);
        card->stations = readStations(*doc, card->sources, where, errors);
        return card;
    } catch (const std::exception& ex) {
        errors.push_back(where + ex.what());
        return nullptr;
    }
}

std::shared_ptr<const Fit> readFit(const std::filesystem::path& file, const std::filesystem::path& library, std::vector<std::string>& errors,
                                   const std::filesystem::path& storeLibrary) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(file, ec)) return nullptr;
    const auto doc = readJson(file, errors);
    if (!doc) return nullptr;
    const std::string where = file.string() + ": ";
    auto fit = std::make_shared<Fit>();
    try {
        fit->aircraft = doc->string("aircraft", "");
        fit->note = doc->string("note", "");
        fit->sources = readSources(*doc);
        if (const core::Json* list = doc->find("payloads"); list && list->isArray())
            for (const auto& p : list->asArray()) {
                FitEntry entry;
                entry.name = p.string("name", "");
                entry.card = p.string("card", "");
                entry.count = static_cast<std::uint16_t>(std::max(1.0, p.number("count", 1.0)));
                entry.source = p.string("source", "");
                entry.note = p.string("note", "");
                const std::string at = where + "payload '" + entry.name + "': ";
                if (entry.name.empty() || entry.card.empty()) {
                    errors.push_back(at + "needs a name and a card");
                    continue;
                }
                if (!fit->sources.count(entry.source)) {
                    errors.push_back(at + "needs a source the fit lists");
                    continue;
                }
                bool duplicate = false;
                for (const auto& other : fit->payloads) duplicate = duplicate || other.name == entry.name;
                if (duplicate) {
                    errors.push_back(at + "a second payload of that name");
                    continue;
                }
                if (!fit->cards.count(entry.card)) {
                    auto card = readCard(library / (entry.card + ".json"), errors);
                    if (!card) {
                        errors.push_back(at + "no card '" + entry.card + "' in " + library.string());
                        continue;
                    }
                    fit->cards[entry.card] = std::move(card);
                }
                fit->payloads.push_back(std::move(entry));
            }
        // the store cards (every one: a station may be loaded with any it takes), then the loadouts
        fit->stores = readStoreLibrary(storeLibrary.empty() ? library.parent_path() / "stores" : storeLibrary, errors);
        if (const core::Json* list = doc->find("loadouts"); list && list->isArray())
            for (const auto& l : list->asArray()) {
                Loadout out;
                out.name = l.string("name", "");
                out.note = l.string("note", "");
                out.source = l.string("source", "");
                const std::string at = where + "loadout '" + out.name + "': ";
                if (out.name.empty()) {
                    errors.push_back(at + "needs a name");
                    continue;
                }
                if (out.note.empty()) errors.push_back(at + "needs a note: what it is for, and its basis");
                if (!out.source.empty() && !fit->sources.count(out.source)) errors.push_back(at + "its source is not one the fit lists");
                if (const core::Json* items = l.find("stations"); items && items->isArray())
                    for (const auto& it : items->asArray()) {
                        Loadout::Item item;
                        item.payload = it.string("payload", "stations");
                        item.station = static_cast<int>(it.number("station", 0.0));
                        item.store = it.string("store", "");
                        item.count = std::max(1, static_cast<int>(it.number("count", 1.0)));
                        const std::string si = at + "station " + std::to_string(item.station) + ": ";
                        const FitEntry* payload = nullptr;
                        for (const auto& p : fit->payloads)
                            if (p.name == item.payload) payload = &p;
                        const auto card = payload ? fit->cards.find(payload->card) : fit->cards.end();
                        const Station* st = card != fit->cards.end() ? card->second->station(item.station) : nullptr;
                        const auto store = fit->stores.find(item.store);
                        if (!st) {
                            errors.push_back(si + "no such station on payload '" + item.payload + "'");
                            continue;
                        }
                        if (store == fit->stores.end()) {
                            errors.push_back(si + "no store card '" + item.store + "'");
                            continue;
                        }
                        if (std::find(st->takes.begin(), st->takes.end(), store->second->storeClass) == st->takes.end() || item.count > st->capacity ||
                            item.count * (store->second->value("mass_kg", 0.0) + store->second->value("fuel_kg", 0.0)) > st->maxKg) {
                            errors.push_back(si + "cannot take " + std::to_string(item.count) + " " + item.store);
                            continue;
                        }
                        out.items.push_back(item);
                    }
                fit->loadouts.push_back(std::move(out));
            }
    } catch (const std::exception& ex) {
        errors.push_back(where + ex.what());
    }
    return fit;
}

std::vector<CapabilityType> capabilitiesOf(Family family) {
    switch (family) {
    case Family::AirRadar: return {CapabilityType::Status, CapabilityType::Amti};
    case Family::WeatherRadar: return {CapabilityType::Status, CapabilityType::WeatherRadar};
    case Family::Sar: return {CapabilityType::Status, CapabilityType::Sar};
    case Family::Smti: return {CapabilityType::Status, CapabilityType::Smti};
    case Family::Esm: return {CapabilityType::Status, CapabilityType::Esm};
    case Family::PassiveOptical: return {CapabilityType::Status, CapabilityType::PassiveOptical};
    case Family::ElectronicAttack: return {CapabilityType::Status, CapabilityType::ElectronicAttack};
    case Family::StoreStation: return {CapabilityType::Status, CapabilityType::StoreManagement, CapabilityType::Strike};
    case Family::Gun: return {CapabilityType::Status, CapabilityType::Strike};
    case Family::Countermeasures: return {CapabilityType::Status, CapabilityType::ElectronicAttack};
    case Family::CommTerminal: return {CapabilityType::Status, CapabilityType::CommTerminal};
    case Family::Pnt: return {CapabilityType::Status, CapabilityType::Pnt};
    case Family::GroundMapRadar: return {CapabilityType::Status}; // (A-GRA has no capability for a real-beam ground map)
    case Family::Count: break;
    }
    return {CapabilityType::Status};
}

control::Support capabilitySupport(CapabilityType type, std::uint8_t& stage) noexcept {
    // ADR-30 section 8: the stage that builds each
    switch (type) {
    case CapabilityType::Status: stage = 0; return control::Support::Supported;
    case CapabilityType::Amti: stage = 4; break;
    case CapabilityType::StoreManagement:
    case CapabilityType::Strike: stage = 7; break;
    case CapabilityType::Esm:
    case CapabilityType::ElectronicAttack: stage = 9; break;
    case CapabilityType::PassiveOptical: stage = 10; break;
    case CapabilityType::WeatherRadar:
    case CapabilityType::Sar:
    case CapabilityType::Smti:
    case CapabilityType::CommTerminal:
    case CapabilityType::Pnt: stage = 11; break;
    case CapabilityType::Count: stage = 0; break;
    }
    return control::Support::NotImplemented;
}

} // namespace fsim::ms
