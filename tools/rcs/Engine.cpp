#include "Engine.h"

#include <vsg/all.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace fsim::rcs {
namespace {

void (*dispatchGate)() = nullptr;

constexpr double kPi = 3.14159265358979323846;
constexpr double kLight = 299792458.0;
constexpr std::uint32_t kGroupSize = 256;
constexpr std::uint32_t kMaxGroupsPerAspect = 1024; ///< each invocation strides over its aspect's rays
constexpr std::uint32_t kMaxGroups = 16384;         ///< workgroups a dispatch (their partial sums read back)
constexpr std::uint32_t kMaxAspects = 256;          ///< aspects a dispatch
constexpr std::uint32_t kRaysPerLane = 32;          ///< a lane's tubes before another workgroup is worth it

// One invocation strides over ray tubes of its aspect (workgroup y); each tube is traced through its bounces, its PO
// current radiated back at each hit, for both incident polarisations at once; the workgroup sums to one partial.
const char* kShader = R"(#version 460
#extension GL_EXT_ray_query : require
layout(local_size_x = 256) in;

layout(set = 0, binding = 0) uniform accelerationStructureEXT scene;
layout(set = 0, binding = 1, std430) readonly buffer Positions { vec4 position[]; };
layout(set = 0, binding = 2, std430) readonly buffer Normals { vec4 normal[]; };
layout(set = 0, binding = 3, std430) readonly buffer Indices { uint index[]; };
struct Aspect {
    vec4 s;      // radar direction (unit, body axes); w: the wavenumber k
    vec4 u;      // the grid's first axis; w: its least coordinate along it
    vec4 v;      // its second axis; w: its least coordinate along it
    vec4 origin; // the start plane's reference point (radar side); w: the tube spacing
    vec4 theta;  // theta-hat; w: the number of tubes along u
    vec4 phi;    // phi-hat; w: the number of tubes along v
};
layout(set = 0, binding = 4, std430) readonly buffer Aspects { Aspect aspects[]; };
layout(set = 0, binding = 5, std430) writeonly buffer Partials { vec4 partial[]; };
layout(set = 0, binding = 6, std430) readonly buffer Kinds { uint kind[]; }; // per triangle: 1 a cavity's interior
layout(push_constant) uniform Params {
    uint groupsPerAspect; uint maxBounces; float phongAlpha; float epsilon;
    float cavity; float skin; float exhaustCavity; float exhaustSkin;
} params;

shared vec4 sumT[256]; // incident theta: (S_tt re, im, S_pt re, im)
shared vec4 sumP[256]; // incident phi:   (S_tp re, im, S_pp re, im)
shared float sumD[256]; // the cavities' diffuse power (m^2)

float sinc(float x) { return abs(x) < 1e-4 ? 1.0 - x * x / 6.0 : sin(x) / x; }
vec3 project(vec3 q, vec3 p, vec3 n) { return q - dot(q - p, n) * n; }

void main() {
    const uint a = gl_WorkGroupID.y;
    const Aspect asp = aspects[a];
    const vec3 s = asp.s.xyz;
    const float k = asp.s.w;
    const float delta = asp.origin.w;
    const uint nu = uint(asp.theta.w), nv = uint(asp.phi.w);
    const uint count = nu * nv;
    const uint stride = params.groupsPerAspect * 256u;
    vec4 accT = vec4(0.0), accP = vec4(0.0);
    float accD = 0.0;

    for (uint i = gl_WorkGroupID.x * 256u + gl_LocalInvocationID.x; i < count; i += stride) {
        const uint iu = i % nu, iv = i / nu;
        vec3 ro = asp.origin.xyz + (asp.u.w + (float(iu) + 0.5) * delta) * asp.u.xyz + (asp.v.w + (float(iv) + 0.5) * delta) * asp.v.xyz;
        vec3 rd = -s;
        // the two incident fields (complex: re, im) and the phase they carry, from the origin's
        vec3 etR = asp.theta.xyz, etI = vec3(0.0), epR = asp.phi.xyz, epI = vec3(0.0);
        float phase = -k * dot(rd, ro);
        for (uint b = 0u; b < params.maxBounces; ++b) {
            rayQueryEXT q;
            rayQueryInitializeEXT(q, scene, gl_RayFlagsOpaqueEXT, 0xFF, ro, params.epsilon, rd, 1e4);
            while (rayQueryProceedEXT(q)) {}
            if (rayQueryGetIntersectionTypeEXT(q, true) == gl_RayQueryCommittedIntersectionNoneEXT) break;
            const float t = rayQueryGetIntersectionTEXT(q, true);
            const int prim = rayQueryGetIntersectionPrimitiveIndexEXT(q, true);
            const vec2 bc = rayQueryGetIntersectionBarycentricsEXT(q, true);
            const uint surface = kind[prim];
            if (surface == 1u || surface == 3u) {
                // into a cavity: its power comes back out diffusely (a Lambertian aperture: 4 rho A per tube of
                // cross-section A), what is left of it after the bounces before
                const float rho = surface == 1u ? params.cavity : params.exhaustCavity;
                accD += rho * 4.0 * delta * delta * 0.5 * (dot(etR, etR) + dot(etI, etI) + dot(epR, epR) + dot(epI, epI));
                break;
            }
            const float skin = surface == 2u ? params.exhaustSkin : params.skin;
            const uint i0 = index[3 * prim], i1 = index[3 * prim + 1], i2 = index[3 * prim + 2];
            const vec3 p0 = position[i0].xyz, p1 = position[i1].xyz, p2 = position[i2].xyz;
            const vec3 n0 = normal[i0].xyz, n1 = normal[i1].xyz, n2 = normal[i2].xyz;
            const float w0 = 1.0 - bc.x - bc.y, w1 = bc.x, w2 = bc.y;
            const vec3 hit = ro + t * rd;
            vec3 ng = normalize(cross(p1 - p0, p2 - p0));
            if (dot(ng, rd) > 0.0) ng = -ng; // (the side the ray meets)
            vec3 ns = normalize(w0 * n0 + w1 * n1 + w2 * n2);
            if (dot(ns, ng) < 0.0) ns = -ns;
            if (dot(ns, rd) > -1e-3) ns = ng; // (an interpolated normal turned away from the ray: the facet's)
            // the radiating point on the Phong surface the facet stands for
            const vec3 phong = w0 * w0 * p0 + w1 * w1 * p1 + w2 * w2 * p2 + w0 * w1 * (project(p1, p0, n0) + project(p0, p1, n1)) +
                               w1 * w2 * (project(p2, p1, n1) + project(p1, p2, n2)) + w2 * w0 * (project(p0, p2, n2) + project(p2, p0, n0));
            const vec3 rp = mix(hit, phong, params.phongAlpha);
            phase -= k * t;
            // the tube's footprint on the facet's plane: a parallelogram of linear phase
            const float cosi = max(-dot(ng, rd), 1e-3);
            const vec3 a1 = normalize(cross(rd, abs(rd.x) < 0.9 ? vec3(1.0, 0.0, 0.0) : vec3(0.0, 1.0, 0.0)));
            const vec3 a2 = cross(rd, a1);
            const vec3 f1 = a1 + rd * (dot(ng, a1) / cosi), f2 = a2 + rd * (dot(ng, a2) / cosi);
            const vec3 qv = k * (s - rd);
            const float amp = delta * delta / cosi * sinc(0.5 * delta * dot(qv, f1)) * sinc(0.5 * delta * dot(qv, f2));
            const float ph = phase + k * dot(s, hit) + dot(qv, rp - hit);
            const vec2 e = vec2(cos(ph), sin(ph)) * (amp * skin); // (a coating's loss on this bounce)
            // PO: 2 n x H with H = rd x E / eta, radiated (the constant k / sqrt(pi) applied on the host)
            const vec3 jtR = cross(ns, cross(rd, etR)), jtI = cross(ns, cross(rd, etI));
            const vec3 jpR = cross(ns, cross(rd, epR)), jpI = cross(ns, cross(rd, epI));
            const vec3 ctR = jtR * e.x - jtI * e.y, ctI = jtR * e.y + jtI * e.x;
            const vec3 cpR = jpR * e.x - jpI * e.y, cpI = jpR * e.y + jpI * e.x;
            accT += vec4(dot(asp.theta.xyz, ctR), dot(asp.theta.xyz, ctI), dot(asp.phi.xyz, ctR), dot(asp.phi.xyz, ctI));
            accP += vec4(dot(asp.theta.xyz, cpR), dot(asp.theta.xyz, cpI), dot(asp.phi.xyz, cpR), dot(asp.phi.xyz, cpI));
            // reflect off the conductor: the tangential field reversed
            vec3 rn = reflect(rd, ns);
            if (dot(rn, ng) <= 0.0) { ns = ng; rn = reflect(rd, ng); }
            etR = skin * (2.0 * dot(ns, etR) * ns - etR); etI = skin * (2.0 * dot(ns, etI) * ns - etI);
            epR = skin * (2.0 * dot(ns, epR) * ns - epR); epI = skin * (2.0 * dot(ns, epI) * ns - epI);
            ro = hit + ng * params.epsilon;
            rd = rn;
        }
    }
    sumT[gl_LocalInvocationID.x] = accT;
    sumP[gl_LocalInvocationID.x] = accP;
    sumD[gl_LocalInvocationID.x] = accD;
    barrier();
    for (uint w = 128u; w > 0u; w >>= 1) {
        if (gl_LocalInvocationID.x < w) {
            sumT[gl_LocalInvocationID.x] += sumT[gl_LocalInvocationID.x + w];
            sumP[gl_LocalInvocationID.x] += sumP[gl_LocalInvocationID.x + w];
            sumD[gl_LocalInvocationID.x] += sumD[gl_LocalInvocationID.x + w];
        }
        barrier();
    }
    if (gl_LocalInvocationID.x == 0u) {
        const uint o = 3u * (a * params.groupsPerAspect + gl_WorkGroupID.x);
        partial[o] = sumT[0];
        partial[o + 1u] = sumP[0];
        partial[o + 2u] = vec4(sumD[0], 0.0, 0.0, 0.0);
    }
}
)";

struct GpuAspect {
    float s[4], u[4], v[4], origin[4], theta[4], phi[4];
};

/// A host-visible storage buffer, mapped.
struct Mapped {
    vsg::ref_ptr<vsg::Buffer> buffer;
    void* data = nullptr;
    VkDeviceSize size = 0;
};

Mapped hostBuffer(vsg::Device* device, VkDeviceSize size, VkBufferUsageFlags usage);

/// A storage buffer in the GPU's own memory, filled from `data` through a staging copy: what every hit reads (the
/// mesh) is not fetched across the bus.
template <class Submit>
vsg::ref_ptr<vsg::Buffer> deviceBuffer(vsg::Device* device, const void* data, VkDeviceSize size, Submit&& submit) {
    Mapped staging = hostBuffer(device, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
    std::memcpy(staging.data, data, size);
    auto buffer = vsg::createBufferAndMemory(device, size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                             VK_SHARING_MODE_EXCLUSIVE, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    submit([&](vsg::CommandBuffer& cb) {
        VkBufferCopy region{0, 0, size};
        vkCmdCopyBuffer(cb, staging.buffer->vk(device->deviceID), buffer->vk(device->deviceID), 1, &region);
    });
    return buffer;
}

Mapped hostBuffer(vsg::Device* device, VkDeviceSize size, VkBufferUsageFlags usage) {
    Mapped m;
    m.size = size;
    m.buffer = vsg::createBufferAndMemory(device, size, usage, VK_SHARING_MODE_EXCLUSIVE,
                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    auto* memory = m.buffer->getDeviceMemory(device->deviceID);
    memory->map(m.buffer->getMemoryOffset(device->deviceID), size, 0, &m.data);
    return m;
}

class VulkanEngine final : public Engine {
public:
    bool init(std::string* error) {
        try {
            instance_ = vsg::Instance::create(vsg::Names{}, vsg::Names{}, VK_API_VERSION_1_2);
            const char* needed[] = {VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, VK_KHR_RAY_QUERY_EXTENSION_NAME,
                                    VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
                                    VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME}; // (core in 1.2, but VSG loads its KHR-named calls)
            vsg::ref_ptr<vsg::PhysicalDevice> chosen;
            for (auto& pd : instance_->getPhysicalDevices()) {
                bool ok = pd->getQueueFamily(VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT) >= 0;
                for (const char* e : needed) ok = ok && pd->supportsDeviceExtension(e);
                if (!ok) continue;
                if (!chosen || pd->getProperties().deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) chosen = pd;
            }
            if (!chosen) {
                if (error) *error = "no Vulkan device with ray queries (VK_KHR_ray_query)";
                return false;
            }
            family_ = chosen->getQueueFamily(VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT);
            name_ = chosen->getProperties().deviceName;
            auto features = vsg::DeviceFeatures::create();
            features->get<VkPhysicalDeviceBufferDeviceAddressFeatures, VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES>()
                .bufferDeviceAddress = VK_TRUE;
            features->get<VkPhysicalDeviceAccelerationStructureFeaturesKHR, VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR>()
                .accelerationStructure = VK_TRUE;
            features->get<VkPhysicalDeviceRayQueryFeaturesKHR, VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR>().rayQuery = VK_TRUE;
            vsg::Names extensions(std::begin(needed), std::end(needed));
            device_ = vsg::Device::create(chosen, vsg::QueueSettings{vsg::QueueSetting{family_, {1.0}}}, vsg::Names{}, extensions, features);
            queue_ = device_->getQueue(static_cast<std::uint32_t>(family_));
            pool_ = vsg::CommandPool::create(device_, static_cast<std::uint32_t>(family_));
            fence_ = vsg::Fence::create(device_);
            return buildPipeline(error);
        } catch (const vsg::Exception& e) {
            if (error) *error = "Vulkan: " + e.message + " (" + std::to_string(e.result) + ")";
        } catch (const std::exception& e) {
            if (error) *error = std::string("Vulkan: ") + e.what();
        }
        return false;
    }

    std::string deviceName() const override { return name_; }
    std::uint64_t raysTraced() const override { return rays_; }

    bool load(const Mesh& mesh, std::string* error) override {
        try {
            const auto nv = static_cast<std::uint32_t>(mesh.positions.size() / 3);
            auto verts = vsg::vec3Array::create(nv);
            for (std::uint32_t i = 0; i < nv; ++i) verts->set(i, vsg::vec3(mesh.positions[3 * i], mesh.positions[3 * i + 1], mesh.positions[3 * i + 2]));
            auto indices = vsg::uintArray::create(static_cast<std::uint32_t>(mesh.indices.size()));
            std::copy(mesh.indices.begin(), mesh.indices.end(), indices->begin());
            auto geometry = vsg::AccelerationGeometry::create();
            geometry->assignVertices(verts);
            geometry->assignIndices(indices);
            blas_ = vsg::BottomLevelAccelerationStructure::create(device_);
            blas_->geometries.push_back(geometry);
            submit([&](vsg::Context& c) { blas_->compile(c); });
            tlas_ = vsg::TopLevelAccelerationStructure::create(device_);
            auto instance = vsg::GeometryInstance::create();
            instance->accelerationStructure = blas_;
            instance->transform = vsg::mat4();
            tlas_->geometryInstances.push_back(instance);
            submit([&](vsg::Context& c) { tlas_->compile(c); });

            std::vector<float> p(nv * 4ull, 0.0f), n(nv * 4ull, 0.0f);
            for (std::uint32_t i = 0; i < nv; ++i)
                for (std::uint32_t k = 0; k < 3; ++k) p[4 * i + k] = mesh.positions[3 * i + k], n[4 * i + k] = mesh.normals[3 * i + k];
            std::vector<std::uint32_t> kd(std::max<std::size_t>(mesh.triangles(), 1), 0u);
            for (std::size_t t = 0; t < mesh.triangles(); ++t) kd[t] = t < mesh.kinds.size() ? mesh.kinds[t] : 0u;
            auto copy = [&](auto&& record) {
                vsg::submitCommandsToQueue(pool_, fence_, 60'000'000'000ull, queue_, record);
                fence_->reset();
            };
            positions_ = {deviceBuffer(device_, p.data(), p.size() * 4ull, copy), p.size() * 4ull};
            normals_ = {deviceBuffer(device_, n.data(), n.size() * 4ull, copy), n.size() * 4ull};
            indices_ = {deviceBuffer(device_, mesh.indices.data(), mesh.indices.size() * 4ull, copy), mesh.indices.size() * 4ull};
            kinds_ = {deviceBuffer(device_, kd.data(), kd.size() * 4ull, copy), kd.size() * 4ull};
            // the target's extent: the corners its rays' grid covers
            lo_[0] = lo_[1] = lo_[2] = 1e30;
            hi_[0] = hi_[1] = hi_[2] = -1e30;
            for (std::uint32_t i = 0; i < nv; ++i)
                for (std::uint32_t k = 0; k < 3; ++k) lo_[k] = std::min(lo_[k], double(mesh.positions[3 * i + k])), hi_[k] = std::max(hi_[k], double(mesh.positions[3 * i + k]));
            hull_.clear();
            sampleHull(mesh);
            bindDescriptors();
            return true;
        } catch (const vsg::Exception& e) {
            if (error) *error = "Vulkan: " + e.message + " (" + std::to_string(e.result) + ")";
        } catch (const std::exception& e) {
            if (error) *error = std::string("load: ") + e.what();
        }
        return false;
    }

    std::vector<Scattering> evaluate(const std::vector<Aspect>& aspects, double f, const Settings& settings) override {
        std::vector<Scattering> out(aspects.size());
        const double k = 2.0 * kPi * f / kLight;
        const double delta = kLight / f / settings.raysPerWavelength;
        const double scale = std::max({hi_[0] - lo_[0], hi_[1] - lo_[1], hi_[2] - lo_[2]});
        const double centre[3] = {(lo_[0] + hi_[0]) / 2, (lo_[1] + hi_[1]) / 2, (lo_[2] + hi_[2]) / 2};
        {
            const float pa = static_cast<float>(settings.phongAlpha), eps = static_cast<float>(std::max(1e-5, scale * 2e-6));
            const float rho = static_cast<float>(settings.cavityReflectivity);
            const float skin = static_cast<float>(std::pow(10.0, -std::max(settings.skinLossDb, 0.0) / 20.0));
            const float exhaustRho = static_cast<float>(settings.exhaustReflectivity);
            const float exhaustSkin = static_cast<float>(std::pow(10.0, -std::max(settings.exhaustLossDb, 0.0) / 20.0));
            std::uint32_t* p = params_->data();
            p[1] = static_cast<std::uint32_t>(settings.maxBounces);
            std::memcpy(p + 2, &pa, 4), std::memcpy(p + 3, &eps, 4), std::memcpy(p + 4, &rho, 4), std::memcpy(p + 5, &skin, 4);
            std::memcpy(p + 6, &exhaustRho, 4), std::memcpy(p + 7, &exhaustSkin, 4);
        }
        // every aspect's grid first: then each dispatch's aspects and workgroups by their tubes - few aspects of many
        // tubes, or many of few (one fence a dispatch: a band's thousands of small aspects were the waiting's)
        std::vector<GpuAspect> grids(aspects.size());
        std::vector<double> tubes(aspects.size());
        for (std::size_t j = 0; j < aspects.size(); ++j) {
            {
                double s[3], th[3], ph[3];
                aspectBasis(aspects[j], s, th, ph);
                // the grid: th and ph span the plane across s; the target's extent projected on them
                double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
                for (const auto& h : hull_) {
                    const double pu = h[0] * th[0] + h[1] * th[1] + h[2] * th[2], pv = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2];
                    umin = std::min(umin, pu), umax = std::max(umax, pu), vmin = std::min(vmin, pv), vmax = std::max(vmax, pv);
                }
                umin -= delta, vmin -= delta, umax += delta, vmax += delta;
                const double nu = std::ceil((umax - umin) / delta), nvv = std::ceil((vmax - vmin) / delta);
                const double back = scale + 1.0 + (centre[0] * s[0] + centre[1] * s[1] + centre[2] * s[2]);
                GpuAspect& a = grids[j];
                for (int c = 0; c < 3; ++c) {
                    a.s[c] = float(s[c]), a.u[c] = float(th[c]), a.v[c] = float(ph[c]), a.theta[c] = float(th[c]), a.phi[c] = float(ph[c]);
                    a.origin[c] = float(s[c] * back);
                }
                a.s[3] = float(k), a.u[3] = float(umin), a.v[3] = float(vmin), a.origin[3] = float(delta);
                a.theta[3] = float(nu), a.phi[3] = float(nvv);
                tubes[j] = nu * nvv;
                rays_ += static_cast<std::uint64_t>(nu * nvv);
            }
        }
        for (std::size_t first = 0; first < aspects.size();) {
            const auto groups = static_cast<std::uint32_t>(
                std::clamp(std::ceil(tubes[first] / (kGroupSize * kRaysPerLane)), 1.0, static_cast<double>(kMaxGroupsPerAspect)));
            const std::size_t n = std::min<std::size_t>({aspects.size() - first, kMaxAspects, kMaxGroups / groups});
            std::memcpy(aspects_.data, grids.data() + first, n * sizeof(GpuAspect));
            params_->data()[0] = groups;
            dispatch_->groupCountX = groups;
            dispatch_->groupCountY = static_cast<std::uint32_t>(n);
            run();
            const auto* p = static_cast<const float*>(partials_.data);
            const double norm = k / std::sqrt(kPi);
            for (std::size_t j = 0; j < n; ++j) {
                double sum[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
                for (std::uint32_t w = 0; w < groups; ++w) {
                    const float* q = p + 12 * (j * groups + w);
                    for (int c = 0; c < 9; ++c) sum[c] += double(q[c]);
                }
                Scattering& s = out[first + j];
                s.tt = std::complex<double>(sum[0], sum[1]) * norm;
                s.pt = std::complex<double>(sum[2], sum[3]) * norm;
                s.tp = std::complex<double>(sum[4], sum[5]) * norm;
                s.pp = std::complex<double>(sum[6], sum[7]) * norm;
                s.diffuse = sum[8];
            }
            first += n;
        }
        return out;
    }

private:
    template <class F>
    void submit(F&& compile) {
        auto context = vsg::Context::create(device_);
        context->graphicsQueue = queue_;
        context->commandPool = pool_;
        compile(*context);
        context->record();
        context->waitForCompletion();
    }

    bool buildPipeline(std::string* error) {
        auto stage = vsg::ShaderStage::create(VK_SHADER_STAGE_COMPUTE_BIT, "main", std::string(kShader));
        auto hints = vsg::ShaderCompileSettings::create();
        hints->vulkanVersion = VK_API_VERSION_1_2;
        hints->target = vsg::ShaderCompileSettings::SPIRV_1_4;
        stage->module->hints = hints;
        auto compiler = vsg::ShaderCompiler::create();
        vsg::ShaderStages stages{stage};
        if (!compiler->supported() || !compiler->compile(stages)) {
            if (error) *error = "the SBR shader did not compile";
            return false;
        }
        vsg::DescriptorSetLayoutBindings bindings{
            {0, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr},
            {6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}};
        setLayout_ = vsg::DescriptorSetLayout::create(bindings);
        layout_ = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{setLayout_}, vsg::PushConstantRanges{{VK_SHADER_STAGE_COMPUTE_BIT, 0, 32}});
        pipeline_ = vsg::BindComputePipeline::create(vsg::ComputePipeline::create(layout_, stage));
        params_ = vsg::uintArray::create(8);
        pushConstants_ = vsg::PushConstants::create(VK_SHADER_STAGE_COMPUTE_BIT, 0, params_);
        dispatch_ = vsg::Dispatch::create(1, 1, 1);
        aspects_ = hostBuffer(device_, sizeof(GpuAspect) * kMaxAspects, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        partials_ = hostBuffer(device_, 48ull * kMaxGroups, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
        return true;
    }

    void bindDescriptors() {
        auto storage = [](const auto& m, std::uint32_t binding) {
            return vsg::DescriptorBuffer::create(vsg::BufferInfoList{vsg::BufferInfo::create(m.buffer.get(), 0, m.size)}, binding, 0,
                                                 VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        };
        auto set = vsg::DescriptorSet::create(setLayout_, vsg::Descriptors{vsg::DescriptorAccelerationStructure::create(vsg::AccelerationStructures{tlas_}, 0),
                                                                           storage(positions_, 1), storage(normals_, 2), storage(indices_, 3),
                                                                           storage(aspects_, 4), storage(partials_, 5),
                                                                           storage(kinds_, 6)});
        bind_ = vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_COMPUTE, layout_, 0, set);
        submit([&](vsg::Context& c) {
            pipeline_->compile(c);
            bind_->compile(c);
        });
    }

    void run() {
        if (dispatchGate) dispatchGate();
        vsg::submitCommandsToQueue(pool_, fence_, 60'000'000'000ull, queue_, [&](vsg::CommandBuffer& cb) {
            pipeline_->record(cb);
            bind_->record(cb);
            pushConstants_->record(cb);
            dispatch_->record(cb);
        });
        fence_->reset();
    }

    /// Points enough to bound the target's projection along any direction: the vertices of a coarse grid's
    /// occupied cells (each cell's corners), so the projected extent errs out by at most a cell.
    void sampleHull(const Mesh& mesh) {
        constexpr int kCells = 24;
        const double size[3] = {std::max(hi_[0] - lo_[0], 1e-6), std::max(hi_[1] - lo_[1], 1e-6), std::max(hi_[2] - lo_[2], 1e-6)};
        std::vector<char> occupied(kCells * kCells * kCells, 0);
        const auto nv = mesh.positions.size() / 3;
        for (std::size_t i = 0; i < nv; ++i) {
            int c[3];
            for (std::size_t k = 0; k < 3; ++k) c[k] = std::clamp(static_cast<int>((mesh.positions[3 * i + k] - lo_[k]) / size[k] * kCells), 0, kCells - 1);
            occupied[static_cast<std::size_t>((c[2] * kCells + c[1]) * kCells + c[0])] = 1;
        }
        for (int z = 0; z < kCells; ++z)
            for (int y = 0; y < kCells; ++y)
                for (int x = 0; x < kCells; ++x)
                    if (occupied[static_cast<std::size_t>((z * kCells + y) * kCells + x)])
                        for (int corner = 0; corner < 8; ++corner)
                            hull_.push_back({lo_[0] + (x + (corner & 1)) * size[0] / kCells, lo_[1] + (y + ((corner >> 1) & 1)) * size[1] / kCells,
                                             lo_[2] + (z + ((corner >> 2) & 1)) * size[2] / kCells});
    }

    vsg::ref_ptr<vsg::Instance> instance_;
    vsg::ref_ptr<vsg::Device> device_;
    vsg::ref_ptr<vsg::Queue> queue_;
    vsg::ref_ptr<vsg::CommandPool> pool_;
    vsg::ref_ptr<vsg::Fence> fence_;
    int family_ = -1;
    std::string name_;
    vsg::ref_ptr<vsg::DescriptorSetLayout> setLayout_;
    vsg::ref_ptr<vsg::PipelineLayout> layout_;
    vsg::ref_ptr<vsg::BindComputePipeline> pipeline_;
    vsg::ref_ptr<vsg::uintArray> params_;
    vsg::ref_ptr<vsg::PushConstants> pushConstants_;
    vsg::ref_ptr<vsg::Dispatch> dispatch_;
    vsg::ref_ptr<vsg::BindDescriptorSet> bind_;
    vsg::ref_ptr<vsg::BottomLevelAccelerationStructure> blas_;
    vsg::ref_ptr<vsg::TopLevelAccelerationStructure> tlas_;
    Mapped aspects_, partials_;
    struct Resident {
        vsg::ref_ptr<vsg::Buffer> buffer;
        VkDeviceSize size = 0;
    };
    Resident positions_, normals_, indices_, kinds_; ///< (the mesh: in the GPU's memory)
    double lo_[3] = {0, 0, 0}, hi_[3] = {0, 0, 0};
    std::vector<std::array<double, 3>> hull_;
    std::uint64_t rays_ = 0;
};

} // namespace

void setDispatchGate(void (*gate)()) { dispatchGate = gate; }

void aspectBasis(const Aspect& a, double s[3], double theta[3], double phi[3]) noexcept {
    const double ca = std::cos(a.azimuthRad), sa = std::sin(a.azimuthRad), ce = std::cos(a.elevationRad), se = std::sin(a.elevationRad);
    s[0] = ce * ca, s[1] = ce * sa, s[2] = -se;
    theta[0] = se * ca, theta[1] = se * sa, theta[2] = ce;
    phi[0] = -sa, phi[1] = ca, phi[2] = 0.0;
}

std::unique_ptr<Engine> Engine::create(std::string* error) {
    auto e = std::make_unique<VulkanEngine>();
    if (!e->init(error)) return nullptr;
    return e;
}

} // namespace fsim::rcs
