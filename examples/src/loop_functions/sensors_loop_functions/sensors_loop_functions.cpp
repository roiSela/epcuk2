/**
 * @file <sensors_loop_functions.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "sensors_loop_functions.h"
#include <argos3/core/simulator/loop_functions.h>

CSensorsLoopFunctions::CSensorsLoopFunctions() {
}

void CSensorsLoopFunctions::Init(TConfigurationNode& t_tree) {
    try {


   } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initialising the loop functions", ex);
    }
}


void CSensorsLoopFunctions::PostStep() {

}

void CSensorsLoopFunctions::PostExperiment() {

}

CColor CSensorsLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
   if(c_position_on_plane.GetX() > -0.05f && c_position_on_plane.GetX() < 0.05f) {
       return CColor::BLACK;
   } else if(c_position_on_plane.GetX() > -0.1f && c_position_on_plane.GetX() < 0.1f) {
      return CColor::GRAY50;
   } else {
      return CColor::WHITE;
   }
}


REGISTER_LOOP_FUNCTIONS(CSensorsLoopFunctions, "sensors_loop_functions")
