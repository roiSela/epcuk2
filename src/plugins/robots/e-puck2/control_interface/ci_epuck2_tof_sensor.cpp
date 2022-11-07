/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.cpp>
 *
 *@author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "ci_epuck2_tof_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   const SInt32 CCI_EPuck2TOFSensor::GetReading() const {
     return m_iReading;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2TOFSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::AddToTable(pt_lua_state, "tof", m_iReading);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2TOFSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_pushnumber(pt_lua_state, m_iReading);
      lua_setfield(pt_lua_state, -2, "tof");
   }
#endif


   /****************************************/
   /****************************************/

}
