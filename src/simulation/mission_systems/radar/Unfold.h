#pragma once
// Temporal ambiguity resolution from raw receiver measurements only.
// Joint feasibility is conservative: competing prior/alias associations or
// shared prior returns leave the current return explicitly unresolved.
#include <array>
#include <vector>
#include <optional>
#include <cmath>
#include <algorithm>

namespace fsim::ms::radar {
struct UnfoldLook {
    double timeS=0;
    std::array<double,3> position{};
    std::array<double,9> covariance{}; // raw receiver position covariance, ECEF
};
struct UnfoldCandidate {
    UnfoldLook look;
    std::array<double,3> direction{},ownVelocity{};
    double rateSigmaMs=0;
    std::vector<std::array<double,2>> aliases;
};

inline std::vector<std::optional<std::array<double,2>>> resolveAliases(
    const std::vector<UnfoldCandidate>& current,const std::vector<UnfoldLook>& previous,
    double maximumSpeedMs,double gate,double processNoise) {
    std::vector<std::optional<std::array<double,2>>> result(current.size());
    std::vector<std::vector<std::size_t>> associations(current.size());
    std::vector<std::size_t> owners(previous.size());
    for(std::size_t i=0;i<current.size();++i) {
        const auto& c=current[i];std::vector<bool> feasible(c.aliases.size());
        for(std::size_t j=0;j<previous.size();++j) {
            const auto& old=previous[j];const double dt=c.look.timeS-old.timeS;if(!(dt>0))continue;
            double distance=0,trace=0,projected=0,rate=0;
            for(std::size_t axis=0;axis<3;++axis) {
                const double d=c.look.position[axis]-old.position[axis];distance+=d*d;
                rate+=(d/dt-c.ownVelocity[axis])*c.direction[axis];
                trace+=c.look.covariance[3*axis+axis]+old.covariance[3*axis+axis];
                for(std::size_t k=0;k<3;++k)projected+=c.direction[axis]*c.direction[k]*
                    (c.look.covariance[3*axis+k]+old.covariance[3*axis+k]);
            }
            if(distance>std::pow(maximumSpeedMs*dt+gate*std::sqrt(std::max(0.,2*trace)),2))continue;
            // Var(A-B)<=2(Var(A)+Var(B)) without inventing independence
            // between reused raw measurements/Doppler interpolation. The
            // declared diffusion bounds the enabled IMM modes here.
            const double variance=4*std::max(0.,projected)/(dt*dt)+2*c.rateSigmaMs*c.rateSigmaMs+processNoise*dt;
            bool supports=false;
            for(std::size_t k=0;k<c.aliases.size();++k)
                if(std::pow(c.aliases[k][1]-rate,2)<=gate*gate*variance)feasible[k]=true,supports=true;
            if(supports)associations[i].push_back(j),++owners[j];
        }
        if(std::count(feasible.begin(),feasible.end(),true)==1)
            result[i]=c.aliases[static_cast<std::size_t>(std::find(feasible.begin(),feasible.end(),true)-feasible.begin())];
    }
    for(std::size_t i=0;i<current.size();++i)
        for(auto j:associations[i])if(owners[j]>1)result[i].reset();
    return result;
}
} // namespace fsim::ms::radar
