#include "world/Detonations.h"

#include "core/Log.h"
#include "world/Blackbody.h"
#include "world/ShadingRate.h"
#include "world/Sky.h"

#include <algorithm>
#include <cmath>

namespace fsim::world {

namespace {

constexpr double kBottomRadius = 6360000.0; // (the clouds' atmosphere, as world::Airflow's)
constexpr double kExposure = 10.0;          // takram's examples' (as the contrails')
constexpr int kStride = 5;                  // rows per detonation (assets/clouds/shaders/detonation.vert)
constexpr double kErosion = 0.6;            // how much the noises cut into the puff's edge

} // namespace

double Detonations::fireballDiameter(double tntKg, double pressurePa) {
    if (!(tntKg > 0.0)) return 0.0;
    return freeAirDiameter1Kg() * std::pow(tntKg, kDiameterExponent) * std::cbrt(101325.0 / std::max(pressurePa, 100.0));
}

Detonations::Burst Detonations::at(double tntKg, double pressurePa, double age) {
    Burst b;
    if (!(tntKg > 0.0) || age < 0.0) return b;
    const double scale = std::cbrt(tntKg);
    const double rMax = 0.5 * fireballDiameter(tntKg, pressurePa);
    const double grow = kGrowS * scale, hold = kHoldS * scale, cool = kCoolS * scale;
    // the fireball: out to its size in some milliseconds; the products' heat at first, then the afterburning's
    // surface, then cooling - its light gone below 900 K (a dull red against the day)
    b.radius = rMax * (1.0 - std::exp(-3.0 * age / grow));
    if (age < grow) b.temperature = kFlashK + (kSurfaceK - kFlashK) * (age / grow);
    else if (age < grow + hold) b.temperature = kSurfaceK;
    else b.temperature = kSurfaceK * std::exp(-(age - grow - hold) / cool);
    if (b.temperature < 900.0) b.temperature = 0.0;
    // the smoke: the soot the afterburning leaves, a puff the fireball's size once it has formed (sigma half its
    // radius: the Gaussian's middle as wide as the fireball was), growing as a decaying turbulent puff
    const double formed = grow + hold;
    const double sigma0 = 0.5 * rMax;
    b.smokeSigma = sigma0 * std::pow(std::max(age, formed) / formed, kPuffExponent);
    // (as the fireball forms its soot appears: none in the flash)
    b.smokeE = kSootShare * tntKg * kSootAbsorption * std::clamp(age / formed, 0.0, 1.0);
    const double tau0 = b.smokeE / (2.0 * 3.141592653589793 * b.smokeSigma * b.smokeSigma);
    b.gone = age > kLongestS || (age > formed && tau0 < kSeen);
    return b;
}

double Detonations::fuelFireballDiameter(double fuelKg, double pressurePa) {
    if (!(fuelKg > 0.0)) return 0.0;
    return kFuelDiameter1Kg * std::cbrt(fuelKg) * std::cbrt(101325.0 / std::max(pressurePa, 100.0));
}

Detonations::Burst Detonations::fuelAt(double fuelKg, double pressurePa, double age) {
    Burst b;
    if (!(fuelKg > 0.0) || age < 0.0) return b;
    const double rMax = 0.5 * fuelFireballDiameter(fuelKg, pressurePa);
    const double duration = kFuelDuration1Kg * std::cbrt(fuelKg);
    // out to its size over its first third; burning at its surface's temperature; fading over its last fifth
    b.radius = rMax * std::min(1.0, std::sqrt(std::max(age, 0.0) / (duration / 3.0)));
    if (age < 0.8 * duration) b.temperature = kFuelK;
    else if (age < duration) b.temperature = kFuelK * (1.0 - 0.45 * (age - 0.8 * duration) / (0.2 * duration));
    if (b.temperature < 900.0) b.temperature = 0.0;
    // its soot, a puff its size once burnt out, growing as the warhead's does
    const double sigma0 = 0.5 * rMax;
    b.smokeSigma = sigma0 * std::pow(std::max(age, duration) / duration, kPuffExponent);
    b.smokeE = kFuelSootYield * fuelKg * kSootAbsorption * std::clamp(age / duration, 0.0, 1.0);
    const double tau0 = b.smokeE / (2.0 * 3.141592653589793 * b.smokeSigma * b.smokeSigma);
    b.gone = age > kLongestS || (age > duration && tau0 < kSeen);
    return b;
}

double Detonations::pressureAt(const Air& air, double altitudeM) {
    // the standard atmosphere's troposphere and lower stratosphere, from the day's sea level
    const double h = std::max(altitudeM, -500.0);
    const double T0 = air.seaLevelK, lapse = 0.0065, g = 9.80665, R = 287.053;
    const double tropopause = 11000.0;
    if (h <= tropopause) return air.seaLevelPa * std::pow(1.0 - lapse * h / T0, g / (R * lapse));
    const double p11 = air.seaLevelPa * std::pow(1.0 - lapse * tropopause / T0, g / (R * lapse));
    return p11 * std::exp(-g * (h - tropopause) / (R * (T0 - lapse * tropopause)));
}

Detonations::Detonations(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders, const AtmosphereTables& tables,
                         bool coarseShading) {
    (void)options;
    root_ = vsg::Group::create();
    switch_ = vsg::Switch::create();
    root_->addChild(switch_);
    const CloudNoise noise = shaders.empty() ? CloudNoise{} : loadCloudNoise(shaders.parent_path());
    if (shaders.empty() || !tables.valid() || !noise.valid()) {
        LOG_WARN("world") << "no detonations drawn: the clouds' shaders, noises or the atmosphere's tables are missing";
        return;
    }
    const std::uint32_t table = static_cast<std::uint32_t>(kMost * kStride);
    const std::string cap = "#define DETONATION_STRIDE " + std::to_string(kStride) + "\n#define BLACKBODY_AT " + std::to_string(table) +
                            "\n#define BLACKBODY_ROWS " + std::to_string(SunlitBlackbody::kRows) + "\n";
    auto vertex = loadCloudShader(shaders, "detonation.vert", VK_SHADER_STAGE_VERTEX_BIT, cap);
    auto fragment = loadCloudShader(shaders, "detonation.frag", VK_SHADER_STAGE_FRAGMENT_BIT, cap);
    vsg::ShaderStages stages{vertex, fragment};
    auto compiler = vsg::ShaderCompiler::create();
    if (!vertex || !fragment || !compiler->supported() || !compiler->compile(stages)) {
        LOG_WARN("world") << "detonation shaders did not build; no detonations drawn";
        return;
    }
    // (the contrails' layout: their shared declarations, contrail.glsl)
    const VkShaderStageFlags both = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    vsg::DescriptorSetLayoutBindings bindings{{0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, both, nullptr},
                                              {1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, both, nullptr}};
    for (std::uint32_t b : {3u, 4u, 6u, 7u, 8u, 20u}) bindings.push_back({b, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, both, nullptr});
    auto setLayout = vsg::DescriptorSetLayout::create(bindings);
    auto layout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout}, vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
    rows_ = vsg::vec4Array::create(table + SunlitBlackbody::kRows, vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
    blackbody_.write(*rows_, table);
    frame_ = vsg::vec4Array::create(4, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    for (auto& d : {rows_, frame_}) d->properties.dataVariance = vsg::DYNAMIC_DATA;
    vsg::Descriptors descriptors{vsg::DescriptorBuffer::create(rows_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                                 vsg::DescriptorBuffer::create(frame_, 1, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)};
    descriptors.push_back(vsg::DescriptorImage::create(noise.shape, 3, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER));
    descriptors.push_back(vsg::DescriptorImage::create(noise.detail, 4, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER));
    for (const auto& d : tables.descriptors()) descriptors.push_back(d);
    auto set = vsg::DescriptorSet::create(setLayout, descriptors);

    // premultiplied over, depth tested and not written; each detonation's box by its far faces (as a contrail's)
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_FRONT_BIT;
    auto assembly = vsg::InputAssemblyState::create();
    assembly->topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
    auto depth = vsg::DepthStencilState::create();
    depth->depthWriteEnable = VK_FALSE;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), assembly, raster, vsg::MultisampleState::create(), blend, depth};
    // shaded a fragment to 2 x 2 pixels on a large window (setScreen), as the contrails are: a puff filling a 4K screen
    // from 60 m took 3.4 ms a frame per pixel
    if (coarseShading) {
        states.push_back(vsg::DynamicState::create(VK_DYNAMIC_STATE_FRAGMENT_SHADING_RATE_KHR));
        shadingRate_ = ShadingRate::create();
    }
    auto group = vsg::StateGroup::create();
    group->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(layout, stages, states)));
    group->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, set));
    draw_ = vsg::Draw::create(14, 0, 0, 0); // (a box a detonation, one strip)
    if (shadingRate_) group->addChild(shadingRate_);
    group->addChild(draw_);
    at_ = vsg::MatrixTransform::create();
    at_->addChild(group);
    switch_->addChild(true, at_);
}

void Detonations::setScreen(std::uint32_t width, std::uint32_t height) {
    screenHeight_ = std::max<std::uint32_t>(height, 1);
    coarse_ = shadingRate_ && static_cast<double>(width) * static_cast<double>(height) >= 2.0e6 ? 2u : 1u;
    if (shadingRate_) static_cast<ShadingRate*>(shadingRate_.get())->size = coarse_;
}

void Detonations::setVisible(bool on) {
    if (on == visible_) return;
    visible_ = on;
    switch_->setAllChildren(on);
    if (!on) bursts_.clear();
}

void Detonations::add(const ipc::EngagementEvent& event, const vsg::EllipsoidModel& ellipsoid) {
    const bool fire = event.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Fire);
    if ((event.kind != static_cast<std::uint8_t>(ipc::EngagementKind::Detonation) && !fire) || !(event.amountKg > 0.0)) return;
    const vsg::dvec3 at(event.positionEcef[0], event.positionEcef[1], event.positionEcef[2]);
    const vsg::dvec3 velocity(event.velocityEcef[0], event.velocityEcef[1], event.velocityEcef[2]);
    // a warhead's: one burst; a fire's spill: released over kReleaseS from the moving aircraft, a chain of lobes
    // behind it, each its share of the fuel (the QF-4B's fireball, stretched aft and lumpy)
    const int lobes = fire ? kReleaseLobes : 1;
    for (int k = 0; k < lobes; ++k) {
        Live b;
        b.fuel = fire;
        const double later = fire ? kReleaseS * k / std::max(lobes - 1, 1) : 0.0;
        b.at = at + velocity * later;
        b.time = event.simTime + later;
        b.tntKg = event.amountKg / lobes;
        b.altitude = ellipsoid.convertECEFToLatLongAltitude(b.at).z;
        b.pressurePa = pressureAt(air_, b.altitude);
        b.seed = std::fmod(std::abs(b.at.x * 0.013 + b.at.y * 0.007 + b.at.z * 0.011) + 13.0 * k, 97.0);
        if (bursts_.size() >= kMost) bursts_.erase(bursts_.begin());
        bursts_.push_back(b);
    }
}

void Detonations::update(double now, const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::EllipsoidModel& ellipsoid,
                         const vsg::dvec3& sunEcef) {
    if (!draw_ || !visible_) return;
    // (a replay's seek back: what had not happened yet is forgotten)
    bursts_.erase(std::remove_if(bursts_.begin(), bursts_.end(),
                                 [&](const Live& b) { return b.time > now + 0.5 || b.burst(now - b.time).gone; }),
                  bursts_.end());
    {
        // the frame, as the contrails': the eye, a pixel's angle, the altitude correction, the sun, the night light
        const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(eye);
        const vsg::dvec3 surface = ellipsoid.convertLatLongAltitudeToECEF(vsg::dvec3(lla.x, lla.y, 0.0));
        const double a2 = ellipsoid.radiusEquator() * ellipsoid.radiusEquator(), b2 = ellipsoid.radiusPolar() * ellipsoid.radiusPolar();
        const vsg::dvec3 normal = vsg::normalize(vsg::dvec3(surface.x / a2, surface.y / a2, surface.z / b2));
        const vsg::dvec3 correction = -(surface - normal * kBottomRadius);
        const vsg::dvec3 sun = vsg::normalize(sunEcef);
        const double pixel = 1.0 / std::max(1.0, 0.5 * std::abs(projection[1][1]) * static_cast<double>(screenHeight_));
        const Daylight d = daylightAt(sunEcef, normal);
        const double sunUp = vsg::dot(sun, vsg::normalize(eye));
        const double night = (1.0 - std::clamp((sunUp + 0.10) / 0.25, 0.0, 1.0)) * d.ambient;
        const auto F = [](double v) { return static_cast<float>(v); };
        frame_->set(0, vsg::vec4(F(eye.x), F(eye.y), F(eye.z), F(pixel * coarse_))); // (a fragment's: coarse_ pixels)
        frame_->set(1, vsg::vec4(F(correction.x), F(correction.y), F(correction.z), F(kBottomRadius)));
        frame_->set(2, vsg::vec4(F(sun.x), F(sun.y), F(sun.z), F(std::fmod(std::max(now, 0.0), 3600.0))));
        frame_->set(3, vsg::vec4(F(d.ambientColour.x * night), F(d.ambientColour.y * night), F(d.ambientColour.z * night), F(kExposure)));
        frame_->dirty();
    }
    const double fromRad = vsg::radians(air_.windFromDeg);
    std::size_t k = 0;
    for (const Live& live : bursts_) {
        const double age = now - live.time;
        if (age < 0.0) continue;
        const Burst b = live.burst(age);
        // carried by the wind (blowing towards from + 180 deg), in the local east-north frame at the burst
        const vsg::dvec3 lla = ellipsoid.convertECEFToLatLongAltitude(live.at);
        const double lat = vsg::radians(lla.x), lon = vsg::radians(lla.y); // (vsg's are degrees)
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        const vsg::dvec3 east(-so, co, 0.0), north(-sl * co, -sl * so, cl);
        const vsg::dvec3 wind = (east * -std::sin(fromRad) + north * -std::cos(fromRad)) * air_.windMs;
        const vsg::dvec3 rel = live.at + wind * age - eye;
        const vsg::dvec4 c = blackbody_.at(b.temperature);
        const double fire = b.temperature > 0.0 ? c.w : 0.0;
        const vsg::dvec3 chroma(c.x, c.y, c.z);
        const auto F = [](double v) { return static_cast<float>(v); };
        const auto row = static_cast<std::uint32_t>(k * kStride);
        rows_->set(row, vsg::vec4(F(rel.x), F(rel.y), F(rel.z), 1.0f));
        rows_->set(row + 1, vsg::vec4(F(b.radius), F(fire), F(b.smokeSigma), F(b.smokeE)));
        rows_->set(row + 2, vsg::vec4(F(chroma.x), F(chroma.y), F(chroma.z), F(kErosion)));
        rows_->set(row + 3, vsg::vec4(F(kSootAlbedo), F(live.seed), F(age), 0.0f));
        // a fuel fireball's flame fluctuates (kFlicker); the warhead's products drawn uniform, as photographed white
        const double flicker = live.fuel && b.temperature > 0.0 ? kFlicker : 0.0;
        rows_->set(row + 4, vsg::vec4(F(flicker > 0.0 ? SunlitBlackbody::meanOfFlicker(b.temperature, flicker) : 0.0), F(flicker), 0.0f, 0.0f));
        if (++k >= kMost) break;
    }
    for (std::size_t j = k; j < kMost; ++j) rows_->set(static_cast<std::uint32_t>(j * kStride), vsg::vec4(0.0f, 0.0f, 0.0f, -1.0f));
    rows_->dirty();
    draw_->instanceCount = static_cast<std::uint32_t>(k);
    at_->matrix = vsg::translate(eye);
}

} // namespace fsim::world
