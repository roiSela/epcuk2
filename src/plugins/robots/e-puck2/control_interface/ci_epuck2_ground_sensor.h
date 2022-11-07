/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_ground_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */
#ifndef CCI_EPUCK2_GROUND_SENSOR_H
#define CCI_EPUCK2_GROUND_SENSOR_H

namespace argos {
   class CCI_Epuck2GroundSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_Epuck2GroundSensor : public CCI_Sensor {

   public:

      virtual ~CCI_Epuck2GroundSensor() {}

      const std::vector<SInt32>& GetReadings() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      std::vector<SInt32> m_tReadings;

   };

}

#endif
