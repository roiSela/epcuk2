/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_encoder_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "ci_epuck2_encoder_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

/****************************************/
/****************************************/

   const CCI_EPuck2EncoderSensor::SReading CCI_EPuck2EncoderSensor::GetReadings() const {
     return m_tReadings;
   }

/****************************************/
/****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2EncoderSensor::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "encoder");
      CLuaUtility::AddToTable(pt_lua_state, "left",  m_tReadings.EncoderLeftWheel );
      CLuaUtility::AddToTable(pt_lua_state, "right", m_tReadings.EncoderRightWheel);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

/****************************************/
/****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2EncoderSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      lua_getfield(pt_lua_state, -1, "encoder");
      lua_pushnumber(pt_lua_state, m_tReadings.EncoderLeftWheel);
      lua_setfield  (pt_lua_state, -2, "left"         );
      lua_pushnumber(pt_lua_state, m_tReadings.EncoderRightWheel);
      lua_setfield  (pt_lua_state, -2, "right"         );
      lua_pop(pt_lua_state, 1);
   }
#endif


/****************************************/
/****************************************/

}
