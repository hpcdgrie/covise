#include "power.h"

namespace core::simulation::power {

void PowerSimulation::computeParameters() {
    computeParameter(m_buses);
    computeParameter(m_generators);
    computeParameter(m_transformators);
}

const std::vector<double> *PowerSimulation::getTimedependentScalar(const std::string &species, const std::string& node) const
{
    if( auto result = Simulation::getTimedependentScalar(m_buses, species, node))
        return result;
    if( auto result = Simulation::getTimedependentScalar(m_generators, species, node))
        return result;
    if( auto result = Simulation::getTimedependentScalar(m_transformators, species, node))
        return result;
    return nullptr;
}

}  // namespace core::simulation::power
