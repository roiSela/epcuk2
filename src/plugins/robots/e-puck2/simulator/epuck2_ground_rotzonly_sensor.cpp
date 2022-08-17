/**
 * @file <argos3/plugins/robots/e-puck2/simulator/e-puck2_ground_rotzonly_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>

#include "epuck2_ground_rotzonly_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static CRange<Real> UNIT(0.0f, 1023.0f);

   /****************************************/
   /****************************************/

   CEPuck2GroundRotZOnlySensor::CEPuck2GroundRotZOnlySensor() :
      m_pcEmbodiedEntity(NULL),
      m_pcFloorEntity(NULL),
      m_pcGroundSensorEntity(NULL),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   void CEPuck2GroundRotZOnlySensor::SetRobot(CComposableEntity& c_entity) {
      m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
      m_pcGroundSensorEntity = &(c_entity.GetComponent<CGroundSensorEquippedEntity>("ground_sensors"));
      m_pcGroundSensorEntity->Enable();
      m_pcFloorEntity = &m_cSpace.GetFloorEntity();
   }

   /****************************************/
   /****************************************/

   void CEPuck2GroundRotZOnlySensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_GroundSensor::Init(t_tree);
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the ground sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         m_tReadings.resize(m_pcGroundSensorEntity->GetNumSensors());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in rotzonly ground sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2GroundRotZOnlySensor::Update() {
      /*
       * We make the assumption that the robot is rotated only wrt to Z
       */
      CRadians cRotZ, cRotY, cRotX;
      /* Set robot center */
      CVector2 cCenterPos;
      /* Position of sensor on the ground after rototranslation */
      CVector2 cSensorPos;
      /* Go through the sensors */
      for(UInt32 i = 0; i < m_tReadings.size(); ++i) {
         CGroundSensorEquippedEntity::SSensor& sSens = m_pcGroundSensorEntity->GetSensor(i);
         /* Get anchor position and orientation */
         cCenterPos.Set(sSens.Anchor.Position.GetX(),
                        sSens.Anchor.Position.GetY());
         sSens.Anchor.Orientation.ToEulerAngles(cRotZ, cRotY, cRotX);
         /* Calculate sensor position on the ground */
         cSensorPos = sSens.Offset;
         cSensorPos.Rotate(cRotZ);
         cSensorPos += cCenterPos;
         /* Get the color */
         const CColor& cColor = m_pcFloorEntity->GetColorAtPoint(cSensorPos.GetX(),
                                                                 cSensorPos.GetY());
         /* Set the reading */
         m_tReadings[i] = cColor.ToGrayScale() * 4.011764706f;
         /* Apply noise to the sensor */
         if(m_bAddNoise) {
            m_tReadings[i] += m_pcRNG->Uniform(m_cNoiseRange);
         }
         UNIT.TruncValue(m_tReadings[i]);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2GroundRotZOnlySensor::Reset() {
      for(UInt32 i = 0; i < GetReadings().size(); ++i) {
         m_tReadings[i] = 0.0f;
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuck2GroundRotZOnlySensor,
                   "epuck2_ground", "rot_z_only",
                   "Daniel H. Stolfi based on Carlo Pinciroli's work",
                   "1.0",
                   "The EPuck2 ground sensor (optimised for 2D).",

                   "This sensor accesses a set of ground sensors. The sensors all return a value\n"
                   "between 0 and 1023, where 0 means black and 1023 means white. Depending on the type\n"
                   "of ground sensor, readings can be a value in between (grayscale sensors). \n"
                   "In controllers, you must include the ci_epuck2_ground_sensor.h header.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_ground implementation=\"rot_z_only\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"

                   "OPTIONAL XML CONFIGURATION\n\n"

                   "It is possible to add uniform noise to the sensors, thus matching the\n"
                   "characteristics of a real robot better. This can be done with the attribute\n"
                   "\"noise_level\", whose allowed range is in [-1,1] and is added to the calculated\n"
                   "reading. The final sensor reading is always normalised in the [0-1] range.\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_ground implementation=\"rot_z_only\"\n"
                   "                       noise_level=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",

                   "Usable"
		  );

}
