/**
 * @file <swarm_loop_functions.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "swarm_loop_functions.h"

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

CSwarmLoopFunctions::CSwarmLoopFunctions() {
}

void CSwarmLoopFunctions::Init(TConfigurationNode& t_tree) {
    try {

        m_v.reserve(GetSpace().GetEntitiesByType("e-puck2").size());

   } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initialising the loop functions", ex);
    }
}


void CSwarmLoopFunctions::PostStep() {
    m_v.clear();
    CSpace::TMapPerType& cEpuckbots = GetSpace().GetEntitiesByType("e-puck2");
    for(CSpace::TMapPerType::iterator it = cEpuckbots.begin();
        it != cEpuckbots.end();
        ++it) {

        CEPuck2Entity& cEpuckBot = *any_cast<CEPuck2Entity*>(it->second);
        CVector3 cPos = cEpuckBot.GetEmbodiedEntity().GetOriginAnchor().Position;
        m_v.push_back(CVector2(cPos.GetX(), cPos.GetY()));
    }
    Real d = 0;
    for (std::vector<CVector2>::iterator it1 = m_v.begin() ; it1 != m_v.end(); ++it1) {
        CVector2 v1 = *it1;
        for (std::vector<CVector2>::iterator it2 = m_v.begin() ; it2 != m_v.end(); ++it2) {
            CVector2 v2 = *it2;
            if (v1 != v2) {
               d += Distance(v1, v2);
            }
        }
    }
    d /= (m_v.size() * (m_v.size()-1));
    LOG << "Avg. Distance: " << d << std::endl;
}

void CSwarmLoopFunctions::PostExperiment() {

}

REGISTER_LOOP_FUNCTIONS(CSwarmLoopFunctions, "swarm_loop_functions")
