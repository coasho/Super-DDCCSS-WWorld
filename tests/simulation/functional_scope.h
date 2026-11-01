#pragma once

// Test population policy only. The ordinary suite retains every family.
// The explicitly authorized SIM-M02 closure scope defers rotor flight cases;
// no command, physical parameter, assertion or numerical threshold is changed.
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace fsim::testscope {
inline bool nonRotorClosure() {
    const char* scope = std::getenv("FSIM_TEST_FUNCTIONAL_SCOPE");
    if (!scope || !*scope) return false;
    if (std::strcmp(scope, "sim-m02-nonrotor-v1") != 0)
        throw std::runtime_error("Unknown test functional population scope");
    return true;
}
inline bool deferRotor(bool rotor = true, const char* population = "rotorcraft") {
    if (!rotor || !nonRotorClosure()) return false;
    std::cout << "FSIM_SCOPE_DEFERRED_ROTOR: " << population << '\n';
    return true;
}
} // namespace fsim::testscope
