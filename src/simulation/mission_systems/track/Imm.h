#pragma once
// Two-mode continuous-time IMM using the existing nonlinear UKF components.
// Mixing and probability update: Bar-Shalom et al. (2001), independently
// comparable with FilterPy IMMEstimator (MIT). No equipment parameters are fitted
// here. q and the mean model sojourn are explicit caller inputs.
#include "simulation/mission_systems/track/Filters.h"
#include <array>
#include <algorithm>
#include <stdexcept>

namespace fsim::ms::track {

class Imm {
public:
    Imm(const Gaussian& initial, const ConstantVelocity& quiet,
        const ConstantVelocity& maneuver, double sojournS)
        : states_{initial, initial}, models_{quiet, maneuver}, sojournS_(sojournS) {
        if (!std::isfinite(sojournS_) || sojournS_ <= 0) throw std::invalid_argument("IMM sojourn must be positive");
        for (const auto& m : models_) for (double q : m.q)
            if (!std::isfinite(q) || q < 0) throw std::invalid_argument("IMM diffusion must be nonnegative");
    }

    void predictTo(double t) {
        if (!std::isfinite(t) || t < states_[0].t) throw std::invalid_argument("IMM prediction time moved backwards");
        const double stay = .5 + .5 * std::exp(-2 * (t - states_[0].t) / sojournS_);
        const double transition[2][2] = {{stay,1-stay},{1-stay,stay}};
        const auto old = states_;
        const auto prior = probabilities_;
        for (std::size_t j=0;j<2;++j) {
            const double c = prior[0]*transition[0][j]+prior[1]*transition[1][j];
            if (!(c>0)) {
                states_[j]=predict(old[j],t,models_[j]);
                probabilities_[j]=0;
                continue;
            }
            states_[j] = predict(reduce({old[0],old[1]},
                {prior[0]*transition[0][j]/c,prior[1]*transition[1][j]/c}),t,models_[j]);
            probabilities_[j] = c;
        }
    }

    bool observe(const Meas& z, const RadarMeasurement& model, Filter filter) {
        std::array<Gaussian,2> posterior;
        std::array<double,2> logs;
        for (std::size_t j=0;j<2;++j) {
            const auto mp = predictMeasurement(states_[j],model,filter);
            MeasCovar l;
            if (!cholesky(mp.s,l) || !update(states_[j],mp,z,posterior[j])) return false;
            const auto d = RadarMeasurement::difference(z,mp.z);
            double logdet=0,quadratic=0;Meas whitened;
            for (std::size_t k=0;k<4;++k) {
                double v=d.a[k];for(std::size_t i=0;i<k;++i)v-=l(k,i)*whitened.a[i];
                whitened.a[k]=v/l(k,k);quadratic+=whitened.a[k]*whitened.a[k];logdet+=2*std::log(l(k,k));
            }
            logs[j]=std::log(probabilities_[j])-.5*(quadratic+logdet+4*std::log(2*kPi));
        }
        const double peak=std::max(logs[0],logs[1]);
        const double a=std::exp(logs[0]-peak),b=std::exp(logs[1]-peak);
        if (!std::isfinite(a+b) || !(a+b>0)) return false;
        states_=posterior;
        // A zero posterior is legal; the next positive-time transition can
        // make that model reachable again. No arbitrary likelihood floor.
        probabilities_={a/(a+b),b/(a+b)};
        return true;
    }

    Gaussian state() const { return reduce({states_[0],states_[1]},{probabilities_[0],probabilities_[1]}); }
    const std::array<Gaussian,2>& modes() const noexcept { return states_; }
    const std::array<double,2>& probabilities() const noexcept { return probabilities_; }

private:
    std::array<Gaussian,2> states_;
    std::array<ConstantVelocity,2> models_;
    std::array<double,2> probabilities_{.5,.5};
    double sojournS_;
};

} // namespace fsim::ms::track
