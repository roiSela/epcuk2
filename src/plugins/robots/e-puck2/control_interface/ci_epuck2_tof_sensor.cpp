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

   const Real CCI_EPuck2TOFSensor::GetReadings() const {
     return m_tReadings;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2TOFSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "tof");
      CLuaUtility::AddToTable(pt_lua_state, 1, m_tReadings);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2TOFSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "tof");
      lua_pushnumber(pt_lua_state, 1          );
      lua_pushnumber(pt_lua_state, m_tReadings);
      lua_settable  (pt_lua_state, -3         );
      lua_pop(pt_lua_state, 1);
   }
#endif


   /****************************************/
   /****************************************/

}
