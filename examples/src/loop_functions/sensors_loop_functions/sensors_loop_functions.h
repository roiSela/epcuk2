/**
 * @file <sensors_loop_functions.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef SENSORS_LOOP_FUNCTIONS_H_
#define SENSORS_LOOP_FUNCTIONS_H_

#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CSensorsLoopFunctions : public CLoopFunctions {

public:

   CSensorsLoopFunctions();
   virtual ~CSensorsLoopFunctions() {}
   virtual void Init(TConfigurationNode& t_tree);
   virtual void PostStep();
   virtual void PostExperiment();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

};


#endif /* SENSORS_LOOP_FUNCTIONS_H_ */
