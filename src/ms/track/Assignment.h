#pragma once
// The rectangular linear sum assignment (ADR-30, MS-5): Crouse's shortest
// augmenting path ("On implementing 2D rectangular assignment algorithms",
// IEEE TAES 52(4), 2016) as SciPy's linear_sum_assignment implements it -
// the solver Stone Soup's GNNWith2DAssignment calls - step for step, so the
// same costs give the same assignment, ties included.

#include <cstddef>
#include <vector>

namespace fsim::ms::track {

/// Minimise the summed cost of assigning each of `rows` rows a distinct column of `cols` (rows <= cols), costs
/// row-major (infinity: not allowed). Returns each row's column, or empty if no assignment is feasible.
std::vector<int> assign(const std::vector<double>& cost, std::size_t rows, std::size_t cols);

} // namespace fsim::ms::track
