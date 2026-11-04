#pragma once
#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "foundation/transport/WeaponThermalBandV1.h"
#include <vsg/all.h>
#include <chrono>
#include <array>
#include <memory>
#include <string>

namespace fsim::ui {
/// Selected-weapon referee diagnostics. All sensor/flight/damage data comes
/// from native snapshots; this command only selects, formats and draws it.
/// setSource/update/record run on the Viewer's render thread.
class WeaponDiagnosticsGui : public vsg::Inherit<vsg::Command,WeaponDiagnosticsGui> {
public:
    struct Source {
        std::string world;
        std::uint64_t publisherPid=0,worldFrame=0;
        std::uint32_t publisherIncarnation=0;
        double simTimeS=0,publisherAgeS=0;
        bool publisherAlive=false,paused=false;
    };
    WeaponDiagnosticsGui()=default;
    void setSource(Source);
    void update();
    bool selectWeapon(std::uint32_t entity,std::uint64_t generation) const;
    unsigned thermalBandCount() const;
    bool selectThermalBand(unsigned band) const;
    /// Companion playback must match the actual fsrec frame, never interpolate
    /// images, navigation or outcomes. Failure clears the previous frame.
    bool setReplayFrame(const ipc::WeaponDiagnosticFrameV1&,std::uint64_t expectedWorldFrame,double expectedSimTimeS);
    void setReplayUnavailable(std::uint64_t expectedWorldFrame,double expectedSimTimeS);
    void clear();
    std::shared_ptr<const ipc::WeaponDiagnosticFrameV1> snapshot() const noexcept{return frame_;}
    void draw() const;
    void record(vsg::CommandBuffer&) const override {draw();}
private:
    void drawThermal(const ipc::WeaponDiagnosticThermalV1&) const;
    void drawRf(const ipc::WeaponDiagnosticRfV1&) const;
    void drawNavigation(const ipc::WeaponDiagnosticNavigationV1&) const;
    void drawGuidanceLink(const ipc::WeaponDiagnosticGuidanceV1&,const ipc::WeaponDiagnosticLinkV1&) const;
    void drawControl(const ipc::WeaponDiagnosticControlV1&) const;
    void drawPropulsion(const ipc::WeaponDiagnosticPropulsionV1&) const;
    void drawEffects(const ipc::WeaponDiagnosticEffectsV1&) const;
    void drawEvents(const ipc::WeaponDiagnosticFrameV1&) const;
    void captureHistory(const ipc::WeaponDiagnosticFrameV1&);
    void drawHistory() const;
    bool stamp(const char*,const ipc::WeaponDiagnosticStampV1&) const;
    Source source_;
    mutable ipc::WeaponDiagnosticsV1 reader_;
    mutable ipc::WeaponThermalBandV1 bandReader_;
    std::shared_ptr<const ipc::WeaponDiagnosticFrameV1> frame_;
    mutable std::uint32_t selectedEntity_=0;
    mutable std::uint64_t selectedGeneration_=0,selectionRequest_=0,lastRequestFrame_=0;
    mutable unsigned selectedBand_=0;
    mutable std::uint64_t lastBandRequestFrame_=0;
    mutable bool segmentation_=false;
    mutable bool expectedPixels_=false;
    bool replay_=false;
    struct History {
        double timeS=0,thrustN=0,fuelKg=0,positionSigmaM=0;
        std::uint32_t availability=0;
    };
    std::array<History,256> history_{};
    mutable std::size_t historyCount_=0,historyNext_=0;
    std::chrono::steady_clock::time_point nextOpen_{};
    std::chrono::steady_clock::time_point nextBandOpen_{};
};
}
