/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_encoder_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef CCI_EPUCK2_ENCODER_SENSOR_H
#define CCI_EPUCK2_ENCODER_SENSOR_H

namespace argos {
   class CCI_EPuck2EncoderSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_EPuck2EncoderSensor : public CCI_Sensor {

   public:

      virtual ~CCI_EPuck2EncoderSensor() {}

      struct SReading
      {
         int EncoderLeftWheel;
         int EncoderRightWheel;

         SReading() :
            EncoderLeftWheel(0),
            EncoderRightWheel(0) {}

         SReading(unsigned u_left_wheel,
                  unsigned u_right_wheel) :
            EncoderLeftWheel(u_left_wheel),
            EncoderRightWheel(u_right_wheel) {}
      };

      const SReading GetReadings() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      SReading m_tReadings;

   };

}

#endif
