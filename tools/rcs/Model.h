#pragma once
// A hangar model (aircraft/<type>/<type>.glb) as the radar sees it: every
// mesh under the scene, its node transforms applied, in the body axes (x
// forward, y right, z down; the CG at the origin, as model3d writes it). Left
// out: the afterburner's plume (its own nodes and materials). The landing gear
// retracted as the viewer moves it (fsim:gear:<deg>[:<g0>:<g1>] at gear
// position 0): the legs folded into their wells, the doors closed over them.
// The control surfaces at neutral. The cavities' interiors - an intake's duct
// and engine face, the gear wells' and recesses' cuts (the "dark" material), a
// nozzle's liner and turbine - marked as such (Mesh::kinds).

#include "Engine.h"

#include <filesystem>
#include <string>
#include <vector>

namespace fsim::rcs {

struct ModelPart {
    std::string name;      ///< its node's
    std::size_t firstTriangle = 0, triangles = 0;
};

struct ModelReport {
    std::size_t meshes = 0, triangles = 0;
    std::size_t skippedNodes = 0, skippedTriangles = 0;
    std::vector<ModelPart> parts; ///< each node with a mesh, in the order read
};

bool readModel(const std::filesystem::path& glb, Mesh& out, ModelReport* report = nullptr, std::string* error = nullptr);

} // namespace fsim::rcs
