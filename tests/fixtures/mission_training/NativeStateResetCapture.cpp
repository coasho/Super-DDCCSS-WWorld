#include "NativeStateReset.h"

#include "session/World.h"
#include "simulation/flight/FlightModel.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <type_traits>
#include <utility>

namespace fsim::test::native_reset {
namespace {

// Compare owned values, never object padding, allocator storage, pointers or borrowed handles.
struct Writer {
    Snapshot values;
    void put(const std::string& key, const std::string& value) { values.emplace(key, value); }
    void put(const std::string& key, const char* value) { put(key, std::string(value ? value : "")); }
    template <typename T> void put(const std::string& key, T value) {
        if constexpr (std::is_enum_v<T>) put(key, static_cast<std::underlying_type_t<T>>(value));
        else if constexpr (std::is_floating_point_v<T>) {
            if (std::isnan(value)) put(key, "nan");
            else if (std::isinf(value)) put(key, std::signbit(value) ? "-inf" : "+inf");
            else {
                std::ostringstream text;
                text.imbue(std::locale::classic());
                text << std::hexfloat << value;
                put(key, text.str());
            }
        } else if constexpr (std::is_signed_v<T>) put(key, std::to_string(static_cast<long long>(value)));
        else put(key, std::to_string(static_cast<unsigned long long>(value)));
    }
    template <typename C> void array(const std::string& key, const C& valuesIn) {
        std::size_t index = 0;
        for (const auto& value : valuesIn) put(key + "/" + std::to_string(index++), value);
        put(key + "/count", index);
    }
    template <typename C, typename F> void list(const std::string& key, const C& valuesIn, F write) {
        std::size_t index = 0;
        for (const auto& value : valuesIn) write(key + "/" + std::to_string(index++), value);
        put(key + "/count", index);
    }
};

#define FIELD(member) w.put(p + "/" #member, v.member)
#define ARRAY(member) w.array(p + "/" #member, v.member)

void state(Writer& w, const std::string& p, const sim::VehicleState& v) {
    FIELD(simTime); ARRAY(positionEcef); ARRAY(attitudeEcefToBody); FIELD(latitudeRad); FIELD(longitudeRad);
    FIELD(altitudeMslM); FIELD(altitudeAglM); ARRAY(eulerRad); ARRAY(velocityBodyMs); ARRAY(velocityNedMs);
    ARRAY(angularRateBodyRadS); ARRAY(accelerationBodyMs2); FIELD(airspeedTrueMs); FIELD(airspeedCalibratedMs);
    FIELD(mach); FIELD(alphaRad); FIELD(betaRad); FIELD(loadFactor); FIELD(aileronRad); FIELD(elevatorRad);
    FIELD(rudderRad); FIELD(flapsRad); FIELD(gearPosition); FIELD(engineCount); FIELD(speedbrakePosition);
    ARRAY(throttlePosition); ARRAY(thrustN); FIELD(fuelKg); FIELD(stepCount); FIELD(onGround); FIELD(diverged);
    FIELD(crashed); ARRAY(rotationBodyToEcef); ARRAY(engineRpm); ARRAY(engineN2); ARRAY(afterburner);
    ARRAY(nozzlePosition); FIELD(leadingEdgeFlapRad); FIELD(wheelCount); ARRAY(wheelCompressionM);
    ARRAY(wheelSteerRad); ARRAY(wheelSpeedMs);
}

void initial(Writer& w, const std::string& p, const sim::InitialConditions& v) {
    FIELD(latitudeDeg); FIELD(longitudeDeg); FIELD(altitudeMslM); FIELD(headingDeg); FIELD(pitchDeg);
    FIELD(rollDeg); FIELD(airspeedTrueMs); FIELD(onGround); ARRAY(velocityNedMs); ARRAY(bodyRatesRadS);
    FIELD(enginesRunning); FIELD(fuelFraction);
}

void environment(Writer& w, const std::string& p, const sim::EnvironmentState& v) {
    FIELD(epochUtcSeconds); FIELD(timeFactor); FIELD(temperatureSeaLevelK); FIELD(pressureSeaLevelPa);
    FIELD(humidity); FIELD(windDirectionDeg); FIELD(windSpeedMs); FIELD(windGustMs); FIELD(turbulence);
    FIELD(visibilityM); FIELD(cloudBaseM); FIELD(cloudCover); FIELD(precipitation); FIELD(revision);
}

void progress(Writer& w, const std::string& p, const control::ActivityProgress& v) {
    FIELD(segment); FIELD(segments); FIELD(segmentId); FIELD(laps); FIELD(percent); FIELD(segmentPercent);
    FIELD(distanceToGoM); FIELD(timeToGoS); FIELD(crossTrackM); FIELD(courseRad); FIELD(headingRad);
    FIELD(altitudeMslM); FIELD(speedMs); FIELD(speedReference);
}

void flightActivity(Writer& w, const std::string& p, const control::ActivityRecord& v) {
    FIELD(id); FIELD(vehicle); FIELD(capability); FIELD(source); FIELD(commandId); FIELD(interactive);
    FIELD(interrupt); FIELD(waiting); FIELD(run); FIELD(runs); FIELD(precedence); FIELD(waitingFor);
    FIELD(axes); FIELD(state); FIELD(reason); FIELD(controller); FIELD(by); FIELD(suggestion);
    FIELD(constraints); FIELD(constraintsSeen); FIELD(startTime); FIELD(endTime);
    w.put(p + "/rank/priority", v.rank.priority); w.put(p + "/rank/precedence", v.rank.precedence);
    w.put(p + "/window/startNotBefore", v.window.startNotBefore); w.put(p + "/window/startNotAfter", v.window.startNotAfter);
    w.put(p + "/window/endNotBefore", v.window.endNotBefore); w.put(p + "/window/endNotAfter", v.window.endNotAfter);
    w.put(p + "/window/criticality", v.window.criticality);
    w.list(p + "/trace", v.trace, [&](const auto& key, const auto& r) {
        w.put(key + "/kind", r.kind); w.put(key + "/id", r.id);
    });
    progress(w, p + "/progress", v.progress);
}

void missionActivity(Writer& w, const std::string& p, const ms::ActivityRecord& v) {
    FIELD(id); FIELD(capability); FIELD(payload); FIELD(source); FIELD(controller); FIELD(commandId);
    FIELD(state); FIELD(reason); FIELD(startTime); FIELD(endTime); FIELD(percent);
}

void store(Writer& w, const std::string& p, const ms::StoreInfo& v) {
    FIELD(index); FIELD(payload); FIELD(station); FIELD(stationName); FIELD(store); FIELD(mnemonic);
    FIELD(storeClass); FIELD(carriage); FIELD(powerable); FIELD(jettisonable); FIELD(flyable); FIELD(status);
    FIELD(massKg); FIELD(fuelKg); FIELD(weapon); FIELD(bayDoors);
}

void strike(Writer& w, const std::string& p, const ms::StrikeStatus& v) {
    FIELD(activity); FIELD(phase); FIELD(jettison); FIELD(target); ARRAY(stores); ARRAY(weapons);
}

void weapon(Writer& w, const std::string& p, const ms::WeaponRecord& v) {
    FIELD(id); FIELD(launcher); FIELD(name); FIELD(type); FIELD(launchTimeS); FIELD(store); FIELD(station);
    FIELD(activity); FIELD(target); FIELD(jettisoned); FIELD(endTimeS); FIELD(end); FIELD(endLatitudeRad);
    FIELD(endLongitudeRad); FIELD(endHeightM); FIELD(endSpeedMs); FIELD(phase); FIELD(seeker); FIELD(holding);
    FIELD(missM); FIELD(burning); FIELD(tracking); FIELD(supported); FIELD(timeToGoS); FIELD(timeToActiveS);
}

void event(Writer& w, const std::string& p, const ms::EngagementEvent& v) {
    FIELD(serial); FIELD(timeS); FIELD(kind); FIELD(reason); FIELD(entity); FIELD(launcher); FIELD(target);
    ARRAY(positionEcef); ARRAY(velocityEcef); FIELD(destroyed);
}

void received(Writer& w, const std::string& p, const comm::DmsReceived& v) {
    FIELD(message); FIELD(source); FIELD(topic); FIELD(net); FIELD(timeSent); FIELD(timeDelivered); ARRAY(bytes);
}

void task(Writer& w, const std::string& p, const comm::ReceivedTask& v) {
    FIELD(from); FIELD(receivedS); FIELD(state); FIELD(percent);
    const auto& o = v.order;
    w.put(p + "/taskId", o.taskId); w.put(p + "/type", o.type); w.put(p + "/priority", o.priority);
    w.put(p + "/latitudeRad", o.latitudeRad); w.put(p + "/longitudeRad", o.longitudeRad);
    w.put(p + "/altitudeM", o.altitudeM); w.put(p + "/radiusM", o.radiusM);
    w.put(p + "/startS", o.startS); w.put(p + "/endS", o.endS); w.put(p + "/targetSource", o.targetSource);
    w.put(p + "/targetPayload", o.targetPayload); w.put(p + "/targetTrack", o.targetTrack); w.array(p + "/text", o.text);
}

void consent(Writer& w, const std::string& p, const comm::PendingConsent& v) {
    FIELD(from); FIELD(receivedS); FIELD(answered);
    const auto& r = v.request;
    w.put(p + "/requestId", r.requestId); w.put(p + "/strike", r.strike); w.array(p + "/store", r.store);
    w.put(p + "/quantity", r.quantity); w.put(p + "/targetPayload", r.targetPayload);
    w.put(p + "/targetSource", r.targetSource); w.put(p + "/targetTrack", r.targetTrack);
    w.put(p + "/latitudeRad", r.latitudeRad); w.put(p + "/longitudeRad", r.longitudeRad);
    w.put(p + "/altitudeM", r.altitudeM); w.put(p + "/timeS", r.timeS); w.put(p + "/reserved", r.reserved);
}

void authority(Writer& w, const std::string& p, const session::MaCore::Authority& v) {
    FIELD(designated); FIELD(designatedModes); FIELD(designatedBy); FIELD(designatedS); FIELD(lostCommsSet);
    FIELD(lostCommsBy); FIELD(lostCommsSetS); FIELD(lost); FIELD(lostSinceS); FIELD(alerts);
    FIELD(packageId); FIELD(leader); ARRAY(members);
    w.put(p + "/lostComms/commandId", v.lostComms.commandId);
    w.put(p + "/lostComms/thresholdS", v.lostComms.thresholdS);
    w.put(p + "/lostComms/activatePlan", v.lostComms.activatePlan);
    w.put(p + "/lostComms/reserved", v.lostComms.reserved); w.put(p + "/lostComms/planId", v.lostComms.planId);
    w.list(p + "/operators", v.operators, [&](const auto& key, const auto& o) {
        w.put(key + "/system", o.system); w.put(key + "/roleId", o.roleId); w.put(key + "/callsign", o.callsign);
    });
    w.list(p + "/commands", v.c2, [&](const auto& key, const auto& c) {
        w.put(key + "/activity", c.activity); w.put(key + "/system", c.system); w.put(key + "/commandId", c.commandId);
    });
}

void netSpec(Writer& w, const std::string& p, const comm::NetSpec& v) {
    FIELD(name); FIELD(rangeM); FIELD(frequencyHz); FIELD(lineOfSight); FIELD(latencyS); FIELD(jitterS);
    FIELD(lossProbability); FIELD(dataRateBps); FIELD(maxQueueS); FIELD(reliableTimeoutS); FIELD(retryS);
    FIELD(maxPayloadBytes); FIELD(headerBytes);
}

void netStats(Writer& w, const std::string& p, const comm::NetStats& v) {
    FIELD(transmissions); FIELD(delivered); FIELD(duplicates); FIELD(outOfRange); FIELD(masked); FIELD(lost);
    FIELD(queueDropped); FIELD(failed); FIELD(overflowed);
}

void position(Writer& w, const std::string& p, const comm::PositionReport& v) {
    FIELD(timeS); FIELD(latitudeRad); FIELD(longitudeRad); FIELD(altitudeMslM); ARRAY(velocityNedMs);
    FIELD(headingRad); FIELD(airspeedTrueMs); FIELD(airspeedCalibratedMs); FIELD(mach);
}

void entityReport(Writer& w, const std::string& p, const comm::EntityReport& v) {
    FIELD(track); FIELD(payload); FIELD(updates); FIELD(timeS); FIELD(latitudeRad); FIELD(longitudeRad);
    FIELD(heightM); ARRAY(velocityNedMs); FIELD(positionSigmaM); FIELD(velocitySigmaMs);
}

void radar(Writer& w, const std::string& p, const ms::RadarStatus& v) {
    FIELD(searching); FIELD(activity); FIELD(mode); FIELD(beamAzimuthRad); FIELD(beamElevationRad);
    FIELD(framePeriodS); FIELD(azimuthCentreRad); FIELD(azimuthHalfWidthRad); FIELD(elevationCentreRad); FIELD(bars);
    FIELD(rangeScaleM); FIELD(azimuthLimitRad); FIELD(elevationLimitRad); FIELD(instrumentedRangeM); FIELD(frequencyHz);
    FIELD(beamwidthAzimuthRad); FIELD(beamwidthElevationRad); FIELD(rotodome); FIELD(electronic); FIELD(dwells);
    FIELD(detections); FIELD(falseAlarms); FIELD(tracks); FIELD(tentativeTracks); FIELD(signalLevel); FIELD(task);
    FIELD(designatedTrack); FIELD(acquiring); FIELD(illuminating); FIELD(trackMemoryS);
}

void track(Writer& w, const std::string& p, const ms::AmtiTrack& v) {
    FIELD(id); FIELD(payload); FIELD(timeS); FIELD(lastUpdateS); FIELD(updates); FIELD(latitudeRad); FIELD(longitudeRad);
    FIELD(heightM); ARRAY(velocityNedMs); FIELD(positionSigmaM); FIELD(velocitySigmaMs); FIELD(rangeM);
    FIELD(azimuthRad); FIELD(elevationRad); FIELD(rangeRateMs); FIELD(truth); ARRAY(stateEcef); ARRAY(covarianceEcef);
}

void dmsEntity(Writer& w, const std::string& prefix, session::World& world, std::uint32_t id,
               const std::set<comm::SystemId>& systems) {
    auto& d = world.dms();
    w.put(prefix + "/system", d.systemOf(id));
    w.put(prefix + "/tail", d.tailNumber(d.systemOf(id)));
    w.put(prefix + "/entityOfSystem", d.entityOf(d.systemOf(id)));
    w.put(prefix + "/subscribedAll", d.subscribedAll(id)); w.array(prefix + "/subscriptions", d.subscriptions(id));
    w.list(prefix + "/nets", d.netsOf(id), [&](const auto& key, const auto& n) {
        w.put(key + "/id", n.id); w.put(key + "/name", n.name); netSpec(w, key + "/spec", n.spec);
    });
    const auto plan = d.plan(id);
    w.put(prefix + "/plan/positionPeriodS", plan.positionPeriodS); w.put(prefix + "/plan/statusPeriodS", plan.statusPeriodS);
    w.put(prefix + "/plan/tracksPeriodS", plan.tracksPeriodS); w.put(prefix + "/plan/peerTimeoutS", plan.peerTimeoutS);
    w.put(prefix + "/plan/trackTimeoutS", plan.trackTimeoutS);
    const auto* inbox = d.inbox(id);
    w.put(prefix + "/inboxPresent", inbox != nullptr);
    if (inbox) w.list(prefix + "/inbox", *inbox, [&](const auto& key, const auto& v) { received(w, key, v); });
    const auto* consents = d.consentsOf(id);
    w.put(prefix + "/consentsPresent", consents != nullptr);
    if (consents) w.list(prefix + "/consents", *consents, [&](const auto& key, const auto& v) { consent(w, key, v); });
    const auto* commands = d.commandStatuses(id);
    w.put(prefix + "/commandsPresent", commands != nullptr);
    if (commands) w.list(prefix + "/commands", *commands, [&](const auto& key, const auto& c) {
        w.put(key + "/from", c.from); w.put(key + "/receivedS", c.receivedS);
        const auto& v = c.status; const auto p = key + "/status";
        FIELD(commandId); FIELD(topic); FIELD(state); FIELD(reason); FIELD(reserved); FIELD(activity); FIELD(timeS); FIELD(percent);
    });
    w.list(prefix + "/peers", d.peers(id), [&](const auto& key, const auto& v) {
        w.put(key + "/system", v.system); w.put(key + "/net", v.net); w.put(key + "/receivedS", v.receivedS);
        w.put(key + "/tailNumber", v.tailNumber); w.put(key + "/hasPosition", v.hasPosition);
        position(w, key + "/position", v.position);
    });
    w.list(prefix + "/peerProvenance", d.peerProvenance(id), [&](const auto& p, const auto& v) {
        FIELD(positionNet); FIELD(positionReceivedS); FIELD(heartbeat); FIELD(heartbeatNet);
        FIELD(heartbeatReceivedS); FIELD(heartbeatTimeS);
    });
    w.list(prefix + "/linkTracks", d.linkTracks(id), [&](const auto& key, const auto& v) {
        w.put(key + "/source", v.source); w.put(key + "/net", v.net); w.put(key + "/receivedS", v.receivedS);
        entityReport(w, key + "/report", v.report);
    });
    for (const auto system : systems) w.put(prefix + "/lastHeard/" + std::to_string(system), d.lastHeard(id, system));
    // The supplied small fixture starts numbering at one and remains within retained history. There is no
    // general enumeration of outgoing/in-flight sends; this does not claim to serialize arbitrary DMS internals.
    std::uint64_t number = 1;
    for (; ; ++number) {
        const auto t = d.transmission(id, number);
        if (!t) break;
        const auto key = prefix + "/transmissions/" + std::to_string(number);
        w.put(key + "/message", t->message); w.put(key + "/topic", t->topic); w.put(key + "/timeSent", t->timeSent);
        w.put(key + "/published", t->published); w.put(key + "/reached", t->reached); w.put(key + "/queueDropped", t->queueDropped);
        w.list(key + "/destinations", t->destinations, [&](const auto& dest, const auto& v) {
            w.put(dest + "/system", v.first); w.put(dest + "/state", v.second);
        });
    }
    w.put(prefix + "/transmissions/contiguousCount", number - 1);
}

} // namespace

Snapshot capture(World& facade) {
    Writer w;
    auto& world = facade.impl();
    w.put("world/time", world.simTime()); w.put("world/worldSteps", world.worldSteps());
    w.put("world/vehicleSteps", world.vehicleSteps()); w.put("world/dt", world.dt());
    w.put("world/frameSkip", world.frameSkip()); w.put("world/published", world.published());
    environment(w, "world/environment", world.environment());
    const auto ids = world.vehicleIds();
    w.array("world/vehicleIds", ids); w.array("world/c2Ids", world.c2Nodes());
    std::set<std::uint32_t> entities(ids.begin(), ids.end());
    const auto c2s = world.c2Nodes(); entities.insert(c2s.begin(), c2s.end());
    for (const auto& a : world.agents()) entities.insert(a.vehicle);
    std::set<comm::SystemId> systems;
    for (const auto id : entities) if (world.dms().systemOf(id)) systems.insert(world.dms().systemOf(id));
    for (const auto id : ids) {
        const auto prefix = "vehicle/" + std::to_string(id);
        if (const auto* info = world.info(id)) {
            const auto& v = *info; const auto p = prefix;
            FIELD(id); FIELD(name); FIELD(type); FIELD(aircraft); FIELD(model); FIELD(alive); FIELD(generation);
            FIELD(role); FIELD(launcher); FIELD(launchTimeS); initial(w, prefix + "/initial", v.initial);
        }
        if (const auto* v = world.vehicleState(id)) state(w, prefix + "/state", *v);
        if (const auto* sensed = world.sensedState(id)) {
            state(w, prefix + "/sensed/state", sensed->state);
            w.put(prefix + "/sensed/gnssValid", sensed->gnssValid); w.put(prefix + "/sensed/airDataValid", sensed->airDataValid);
            w.put(prefix + "/sensed/positionErrorM", sensed->positionErrorM);
        }
        if (const auto* inputs = world.inputs(id)) {
            const auto& v = *inputs; const auto p = "vehicle/" + std::to_string(id) + "/inputs";
            FIELD(aileron); FIELD(elevator); FIELD(rudder); ARRAY(throttle); FIELD(flaps); FIELD(gearDown);
            FIELD(brakeLeft); FIELD(brakeRight);
        }
        w.put(prefix + "/controlMode", world.controlMode(id)); w.put(prefix + "/default", world.vehicleDefault(id));
        w.put(prefix + "/protection", world.protection(id));
        w.list(prefix + "/activities", world.activities(id), [&](const auto& key, const auto& v) { flightActivity(w, key, v); });
        w.list(prefix + "/plans", world.plans(id), [&](const auto& p, const auto& v) {
            FIELD(id); FIELD(version); FIELD(revision); FIELD(state); FIELD(execution); FIELD(reason);
            FIELD(forPlanningUseOnly); FIELD(activity); FIELD(percent); FIELD(startTime); FIELD(endTime); FIELD(commandId); FIELD(faOwned);
        });
        if (const auto* model = world.model(id)) {
            const auto v = model->energy(); const auto p = prefix + "/energy";
            FIELD(fuelKg); FIELD(fuelCapacityKg); FIELD(fuelFlowKgS); FIELD(chargeJ); FIELD(chargeCapacityJ);
            FIELD(powerW); FIELD(massKg); FIELD(starved);
            // Virtual enumeration stays in the flight-model module and reads tied numeric values without
            // PropertyHandle::get linkage or mutation of the property tree.
            for (const char* propertyRoot : {"propulsion", "fcs", "fsim/damage"})
                for (const auto& [path, value] : model->properties(propertyRoot))
                    w.put(prefix + "/model/" + propertyRoot + "/" + path, value);
        }
        const auto* damage = world.damage(id);
        w.put(prefix + "/damage/present", damage != nullptr);
        if (damage) {
            w.put(prefix + "/damage/crewLost", damage->crewLost());
            w.array(prefix + "/damage/lostChannels", damage->lostChannels());
            if (const auto* s = world.vehicleState(id))
                for (int engine = 0; engine < s->engineCount; ++engine)
                    w.put(prefix + "/damage/engine/" + std::to_string(engine), damage->engineOut(engine));
            for (const auto& [index, v] : damage->tanks()) {
                const auto p = "vehicle/" + std::to_string(id) + "/damage/tank/" + std::to_string(index);
                FIELD(holeM2); FIELD(burning); FIELD(component);
            }
            w.put(prefix + "/damage/tanks/count", damage->tanks().size());
            w.list(prefix + "/damage/sections", damage->sections(), [&](const auto& p, const auto& v) {
                FIELD(strength); FIELD(lift); FIELD(separated); FIELD(tornM2); FIELD(holeM2); FIELD(removed); FIELD(kick);
            });
            for (std::size_t panel = 0; panel < damage->airframe().panels.size(); ++panel)
                w.put(prefix + "/damage/panel/" + std::to_string(panel), damage->panelTorn(static_cast<int>(panel)));
        }
        const auto* host = world.missionSystems(id);
        w.put(prefix + "/mission/present", host != nullptr);
        if (host) {
            w.list(prefix + "/mission/payloads", host->payloads(), [&](const auto& p, const auto& v) {
                FIELD(index); FIELD(name); FIELD(card); FIELD(designation); FIELD(summary); FIELD(family);
                FIELD(state); FIELD(count); FIELD(source);
            });
            for (const auto& payload : host->payloads()) if (payload.family == ms::Family::AirRadar) {
                const auto key = prefix + "/radar/" + std::to_string(payload.index);
                radar(w, key + "/status", host->radarStatus(payload.index));
                if (const auto* own = world.vehicleState(id))
                    w.list(key + "/tracks", host->amtiTracks(payload.index, world.simTime(), *own),
                           [&](const auto& item, const auto& v) { track(w, item, v); });
            }
            w.list(prefix + "/mission/stores", host->stores(), [&](const auto& key, const auto& v) { store(w, key, v); });
            w.list(prefix + "/mission/activities", host->activities(), [&](const auto& key, const auto& v) {
                missionActivity(w, key, v);
                const auto status = host->strikeStatus(v.id);
                w.put(key + "/strike/present", status.has_value());
                if (status) strike(w, key + "/strike", *status);
            });
            w.list(prefix + "/mission/capabilities", host->capabilities(), [&](const auto& key, const auto& v) {
                w.put(key + "/index", v.index); w.put(key + "/payload", v.payload); w.put(key + "/type", v.type);
                w.put(key + "/id", v.id); w.put(key + "/support", v.support); w.put(key + "/stage", v.stage);
                w.put(key + "/missing", v.missing);
                const auto cs = host->controlStatus(v.index);
                w.put(key + "/control/allowed", cs.allowed); w.put(key + "/control/granted", cs.granted);
                w.put(key + "/control/holder", cs.holder);
                const auto availability = host->status(v.index);
                w.put(key + "/availability", availability.availability); w.put(key + "/reason", availability.reason);
                w.put(key + "/nextAvailableS", availability.nextAvailableS);
            });
        }
        const auto inventory = world.countermeasures(id);
        w.put(prefix + "/countermeasures/chaff", inventory.chaff); w.put(prefix + "/countermeasures/flare", inventory.flare);
        w.list(prefix + "/designations", world.designations(id), [&](const auto& p, const auto& v) {
            FIELD(id); FIELD(owner); FIELD(payload); FIELD(track); FIELD(source); FIELD(valid);
            state(w, p + "/estimate", v.estimate);
        });
    }
    w.list("weapons", world.weapons(), [&](const auto& key, const auto& v) { weapon(w, key, v); });
    w.list("decoys", world.decoys(), [&](const auto& p, const auto& v) {
        FIELD(id); FIELD(owner); FIELD(flare); FIELD(ageS); ARRAY(positionEcef);
    });
    w.list("engagementEvents", world.engagementEvents(), [&](const auto& key, const auto& v) { event(w, key, v); });
    for (const auto id : c2s) if (const auto* node = world.c2Node(id)) {
        const auto p = "c2/" + std::to_string(id); const auto& v = *node;
        FIELD(name); FIELD(latitudeRad); FIELD(longitudeRad); FIELD(altitudeMslM);
    }
    w.array("dms/entities", world.dms().entities()); w.array("dms/coreWaiting", world.dms().coreWaiting());
    w.list("dms/nets", world.dms().nets(), [&](const auto& key, const auto& n) {
        w.put(key + "/id", n.id); w.put(key + "/name", n.name); netSpec(w, key + "/spec", n.spec);
        netStats(w, key + "/stats", world.dms().stats(n.id));
    });
    for (const auto id : entities) {
        const auto prefix = "entity/" + std::to_string(id);
        dmsEntity(w, prefix + "/dms", world, id, systems);
        const auto* a = world.maCore().authority(id);
        w.put(prefix + "/authority/present", a != nullptr);
        if (a) authority(w, prefix + "/authority", *a);
        const auto* tasks = world.maCore().tasks(id);
        w.put(prefix + "/tasks/present", tasks != nullptr);
        if (tasks) w.list(prefix + "/tasks", *tasks, [&](const auto& key, const auto& v) { task(w, key, v); });
        const auto* overrides = world.maCore().overrides(id);
        w.put(prefix + "/overrides/present", overrides != nullptr);
        if (overrides) w.list(prefix + "/overrides", *overrides, [&](const auto& p, const auto& v) {
            FIELD(from); FIELD(kind); FIELD(value); FIELD(expiryS);
        });
        const auto rbac = world.maCore().rbac(id);
        w.put(prefix + "/rbac/strikePolicy", rbac.strikePolicy);
        w.list(prefix + "/rbac/roles", rbac.roles, [&](const auto& p, const auto& v) {
            FIELD(id); FIELD(name); FIELD(permissions); FIELD(approvesStrikes);
        });
    }
    w.list("agents", world.agents(), [&](const auto& p, const auto& v) {
        FIELD(vehicle); FIELD(controller); FIELD(name); ARRAY(calls); ARRAY(refused);
    });
    w.list("services", world.maServices().all(), [&](const auto& p, const auto& v) {
        FIELD(serviceId); FIELD(platform); FIELD(controller); ARRAY(capabilities); FIELD(ownsDmsIngress); FIELD(ownsDmsEgress);
    });
    return std::move(w.values);
}

#undef ARRAY
#undef FIELD

} // namespace fsim::test::native_reset
