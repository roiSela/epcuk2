/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_proximity_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Danesh Tarapore's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef CCI_EPUCK2_PROXIMITY_SENSOR_H
#define CCI_EPUCK2_PROXIMITY_SENSOR_H

namespace argos {
   class CCI_EPuck2ProximitySensor;
}

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_EPuck2ProximitySensor : public CCI_Sensor {

   public:

      CCI_EPuck2ProximitySensor();
      virtual ~CCI_EPuck2ProximitySensor() {}

      struct SReading
      {
         SInt32 Value;
         CRadians Angle;

         SReading() :
            Value(0.0f) {}

         SReading(SInt32 f_value,
                  const CRadians& c_angle) :
            Value(f_value),
            Angle(c_angle) {}
      };

      typedef std::vector<SReading> TReadings;



      inline const TReadings& GetReadings() const
      {
         return m_tReadings;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TReadings m_tReadings;
   };

}

#endif
