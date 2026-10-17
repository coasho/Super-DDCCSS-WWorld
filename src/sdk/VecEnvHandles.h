#pragma once

// Private C VecEnv handle shared by the C owner and C++ wrapper.
#include "training/native/vecenv/VecEnv.h"
#include "sdk/Handles.h"
#include <memory>

struct fsim_vecenv {
    fsim::env::VecEnv env;
    fsim::env::VecEnv::StepResult last{};
    std::unique_ptr<fsim_world> world; ///< borrowed handle for fsim_vecenv_world(), made on demand
    explicit fsim_vecenv(const fsim::env::Scenario& s, const fsim::env::VecEnv::Options& o) : env(s, o) {}
};
