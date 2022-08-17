/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's works
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */


#include "ci_epuck2_leds_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

    // TODO: LUA

#ifdef ARGOS_WITH_LUA
   /*
    * This function expects the stack to have either two or four arguments.
    * The first argument must always be the index of the LED to set.
    * Then, in case two arguments are passed, the second argument can be the string
    * definition of a color. In case of four arguments, the RGB values are expected.
    */
   int LuaLEDSetSingleColor(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 2 && lua_gettop(pt_lua_state) != 4) {
         return luaL_error(pt_lua_state, "robot.leds.set_single_color() expects 2 or 4 arguments");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TNUMBER);
      size_t unIdx = lua_tonumber(pt_lua_state, 1);
      /* Get reference to actuator */
      CCI_EPuck2LEDsActuator* pcAct = CLuaUtility::GetDeviceInstance<CCI_EPuck2LEDsActuator>(pt_lua_state, "leds");
      if(unIdx < 1 || unIdx > pcAct->GetNumLEDs()) {
         return luaL_error(pt_lua_state, "passed index %d out of bounds [1,%d]", unIdx, pcAct->GetNumLEDs());
      }
      /* Create color buffer */
      CColor cColor;
      if(lua_gettop(pt_lua_state) == 2) {
         luaL_checktype(pt_lua_state, 2, LUA_TSTRING);
         try {
            cColor.Set(lua_tostring(pt_lua_state, 2));
         }
         catch(CARGoSException& ex) {
            return luaL_error(pt_lua_state, ex.what());
         }
      }
      else {
         luaL_checktype(pt_lua_state, 2, LUA_TNUMBER);
         luaL_checktype(pt_lua_state, 3, LUA_TNUMBER);
         luaL_checktype(pt_lua_state, 4, LUA_TNUMBER);
         cColor.Set(lua_tonumber(pt_lua_state, 2),
                    lua_tonumber(pt_lua_state, 3),
                    lua_tonumber(pt_lua_state, 4));
      }
      /* Perform action */
      // TODO: pcAct->SetSingleColor(unIdx - 1, cColor);
      return 0;
   }

   /*
    * This function expects the stack to have either one or three arguments.
    * In case one argument is passed, it must be the string definition of a color.
    * In case of three arguments, the RGB values are expected.
    */
   int LuaLEDSetAllColors(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 1 && lua_gettop(pt_lua_state) != 3) {
         return luaL_error(pt_lua_state, "robot.leds.set_all_colors() expects 1 or 3 arguments");
      }
      /* Create color buffer */
      CColor cColor;
      if(lua_gettop(pt_lua_state) == 1) {
         luaL_checktype(pt_lua_state, 1, LUA_TSTRING);
         try {
            cColor.Set(lua_tostring(pt_lua_state, 1));
         }
         catch(CARGoSException& ex) {
            return luaL_error(pt_lua_state, ex.what());
         }
      }
      else {
         luaL_checktype(pt_lua_state, 1, LUA_TNUMBER);
         luaL_checktype(pt_lua_state, 2, LUA_TNUMBER);
         luaL_checktype(pt_lua_state, 3, LUA_TNUMBER);
         cColor.Set(lua_tonumber(pt_lua_state, 1),
                    lua_tonumber(pt_lua_state, 2),
                    lua_tonumber(pt_lua_state, 3));
      }
      /* Perform action */
      // TODO CLuaUtility::GetDeviceInstance<CCI_EPuck2LEDsActuator>(pt_lua_state, "leds")->SetAllColors(cColor);
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

   size_t CCI_EPuck2LEDsActuator::GetNumLEDs() const {
     return m_tSettings.size();
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRedLed1(const bool b_state) {
       m_tSettings[0] = b_state ? CColor::RED : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRedLed3(const bool b_state) {
       m_tSettings[2] = b_state ? CColor::RED : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRedLed5(const bool b_state) {
       m_tSettings[4] = b_state ? CColor::RED : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRedLed7(const bool b_state) {
       m_tSettings[6] = b_state ? CColor::RED : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetAllRedLeds(const bool b_state) {
       m_tSettings[0] = b_state ? CColor::RED : CColor::BLACK;
       m_tSettings[2] = b_state ? CColor::RED : CColor::BLACK;
       m_tSettings[4] = b_state ? CColor::RED : CColor::BLACK;
       m_tSettings[6] = b_state ? CColor::RED : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRGBLed2Color(const CColor& c_color) {
      m_tSettings[1] = c_color;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRGBLed4Color(const CColor& c_color) {
      m_tSettings[3] = c_color;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRGBLed6Color(const CColor& c_color) {
      m_tSettings[5] = c_color;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetRGBLed8Color(const CColor& c_color) {
      m_tSettings[7] = c_color;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetAllRGBColors(const CColor& c_color) {
       m_tSettings[1] = c_color;
       m_tSettings[3] = c_color;
       m_tSettings[5] = c_color;
       m_tSettings[7] = c_color;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetFrontLed(const bool b_state) {
       m_tSettings[8] = b_state ? CColor::RED : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetBodyLed(const bool b_state) {
       m_tSettings[9] = b_state ? CColor::GREEN : CColor::BLACK;
   }

   /****************************************/
   /****************************************/

   void CCI_EPuck2LEDsActuator::SetAllBlack() {
       for(size_t i = 0; i < m_tSettings.size(); ++i) {
           m_tSettings[i] = CColor::BLACK;
       }
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_EPuck2LEDsActuator::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "leds");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "set_single_color", &LuaLEDSetSingleColor);
      CLuaUtility::AddToTable(pt_lua_state, "set_all_colors", &LuaLEDSetAllColors);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

}
