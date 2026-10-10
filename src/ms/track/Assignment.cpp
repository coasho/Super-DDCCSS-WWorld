#include "ms/track/Assignment.h"

#include <algorithm>
#include <limits>
#include <utility>

namespace fsim::ms::track {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// SciPy's augmenting_path (scipy/optimize/rectangular_lsap/rectangular_lsap.cpp)
long augmentingPath(std::size_t nc, const std::vector<double>& cost, std::vector<double>& u, std::vector<double>& v,
                    std::vector<long>& path, std::vector<long>& row4col, std::vector<double>& shortest, long i,
                    std::vector<char>& sr, std::vector<char>& sc, std::vector<long>& remaining, double& minVal) {
    long numRemaining = static_cast<long>(nc);
    // (filled in reverse, so that columns are scanned in order when costs tie)
    for (std::size_t it = 0; it < nc; ++it) remaining[it] = static_cast<long>(nc - it - 1);
    std::fill(sr.begin(), sr.end(), 0);
    std::fill(sc.begin(), sc.end(), 0);
    std::fill(shortest.begin(), shortest.end(), kInf);
    long sink = -1;
    while (sink == -1) {
        long index = -1;
        double lowest = kInf;
        sr[static_cast<std::size_t>(i)] = 1;
        for (long it = 0; it < numRemaining; ++it) {
            const auto j = static_cast<std::size_t>(remaining[static_cast<std::size_t>(it)]);
            const double r = minVal + cost[static_cast<std::size_t>(i) * nc + j] - u[static_cast<std::size_t>(i)] - v[j];
            if (r < shortest[j]) {
                path[j] = i;
                shortest[j] = r;
            }
            // (the lowest; on a tie, a column not yet assigned: it ends the path)
            if (shortest[j] < lowest || (shortest[j] == lowest && row4col[j] == -1)) {
                lowest = shortest[j];
                index = it;
            }
        }
        minVal = lowest;
        if (minVal == kInf) return -1; // (infeasible)
        const auto j = static_cast<std::size_t>(remaining[static_cast<std::size_t>(index)]);
        if (row4col[j] == -1) sink = static_cast<long>(j);
        else i = row4col[j];
        sc[j] = 1;
        remaining[static_cast<std::size_t>(index)] = remaining[static_cast<std::size_t>(--numRemaining)];
    }
    return sink;
}

} // namespace

std::vector<int> assign(const std::vector<double>& cost, std::size_t nr, std::size_t nc) {
    if (nr == 0) return {};
    if (nr > nc || cost.size() != nr * nc) return {};
    std::vector<double> u(nr, 0.0), v(nc, 0.0), shortest(nc, kInf);
    std::vector<long> path(nc, -1), col4row(nr, -1), row4col(nc, -1), remaining(nc);
    std::vector<char> sr(nr), sc(nc);
    for (std::size_t cur = 0; cur < nr; ++cur) {
        double minVal = 0.0;
        const long sink = augmentingPath(nc, cost, u, v, path, row4col, shortest, static_cast<long>(cur), sr, sc, remaining, minVal);
        if (sink < 0) return {};
        // update the dual variables
        u[cur] += minVal;
        for (std::size_t i = 0; i < nr; ++i)
            if (sr[i] && i != cur) u[i] += minVal - shortest[static_cast<std::size_t>(col4row[i])];
        for (std::size_t j = 0; j < nc; ++j)
            if (sc[j]) v[j] -= minVal - shortest[j];
        // augment the previous solution
        long j = sink;
        for (;;) {
            const long i = path[static_cast<std::size_t>(j)];
            row4col[static_cast<std::size_t>(j)] = i;
            std::swap(col4row[static_cast<std::size_t>(i)], j);
            if (static_cast<std::size_t>(i) == cur) break;
        }
    }
    std::vector<int> out(nr);
    for (std::size_t i = 0; i < nr; ++i) out[i] = static_cast<int>(col4row[i]);
    return out;
}

} // namespace fsim::ms::track
