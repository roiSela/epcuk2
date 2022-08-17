/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Danesh Tarapore's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef CCI_EPUCK2_TOF_SENSOR_H
#define CCI_EPUCK2_TOF_SENSOR_H

namespace argos {
   class CCI_EPuck2TOFSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CCI_EPuck2TOFSensor : public CCI_Sensor {

   public:

      virtual ~CCI_EPuck2TOFSensor() {}

      const Real GetReadings() const;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      Real m_tReadings;

   };

}

#endif
