/**
 * @file <swarm_loop_functions.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef SWARM_LOOP_FUNCTIONS_H_
#define SWARM_LOOP_FUNCTIONS_H_

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CSwarmLoopFunctions : public CLoopFunctions {

public:

   CSwarmLoopFunctions();
   virtual ~CSwarmLoopFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();

private:
   std::vector<CVector2> m_v;
};


#endif /* SWARM_LOOP_FUNCTIONS_H_ */
