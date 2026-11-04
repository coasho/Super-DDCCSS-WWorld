#pragma once

// Test-only producer provenance. No simulation parameter, equation or SDK
// interface is defined here. Observe only modules actually loaded in THIS
// native process; an adjacent unused DLL is not reported as executed code.
#include "foundation/core/Json.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cwctype>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace fsim::testing {
template<class Hash>
core::Json componentReferenceRuntimeV1(Hash hash,bool requireJsbsim=false) {
    core::Json::Array observed;
#if defined(_WIN32)
    bool jsbsim=false;
    for(const auto* name:{L"libJSBSim.dll",L"libfsim.dll"}) {
        const auto module=GetModuleHandleW(name);
        if(!module)continue;
        std::vector<wchar_t> buffer(32768);
        const auto count=GetModuleFileNameW(module,buffer.data(),static_cast<DWORD>(buffer.size()));
        REQUIRE(count>0);REQUIRE(count<buffer.size());
        const std::filesystem::path path(std::wstring(buffer.data(),count));
        for(const auto& part:path) {
            auto lower=part.wstring();
            std::transform(lower.begin(),lower.end(),lower.begin(),[](wchar_t c){return static_cast<wchar_t>(std::towlower(c));});
            REQUIRE(lower!=L".codex");
        }
        core::Json identity;identity.set("path",path.generic_string()).set("sha256",hash(path));
        observed.push_back(std::move(identity));
        if(std::wstring(name)==L"libJSBSim.dll")jsbsim=true;
    }
    if(requireJsbsim)REQUIRE(jsbsim);
#else
    static_cast<void>(hash);static_cast<void>(requireJsbsim);
    FAIL("Loaded component-reference module provenance requires the maintained Windows presets");
#endif
    return core::Json(std::move(observed));
}
} // namespace fsim::testing
