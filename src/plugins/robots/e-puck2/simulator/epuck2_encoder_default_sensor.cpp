/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_encoder_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_encoder_default_sensor.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

#include "epuck2_encoder_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CEPuck2EncoderDefaultSensor::CEPuck2EncoderDefaultSensor() :
      m_pcEmbodiedEntity(NULL),
      m_pcEncoderEntity(NULL),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_fLeft(0.0),
      m_fRight(0.0) {}

   /****************************************/
   /****************************************/

   void CEPuck2EncoderDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
         m_pcEncoderEntity = &(c_entity.GetComponent<CEPuck2EncoderEquippedEntity>("encoder_sensor"));
         m_pcEncoderEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the encoder default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_EPuck2EncoderSensor::Init(t_tree);
         m_fLeft = 0.0;
         m_fRight = 0.0;
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
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default encoder sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderDefaultSensor::Update() {
      /* Go through the sensors */
      if (m_pcEncoderEntity->GetNumSensors() > 0) {
         CEPuck2EncoderEquippedEntity::SSensor& sSens = m_pcEncoderEntity->GetSensor(0);
         m_fLeft += sSens.StepsPerCm * sSens.Wheels.GetWheelVelocity(0) / CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick();
         m_fRight += sSens.StepsPerCm * sSens.Wheels.GetWheelVelocity(1) / CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick();

         /* Apply noise to the sensor */
         if(m_bAddNoise) {
            m_fLeft += m_pcRNG->Uniform(m_cNoiseRange)*32767.0;
            m_fRight += m_pcRNG->Uniform(m_cNoiseRange)*32767.0;
         }
      } else {
            m_fLeft = 0.0;
            m_fRight = 0.0;
      }

      if (m_fLeft >= 32768.0) {
         m_fLeft -= 65536.0;
      } else if (m_fLeft < -32768.0) {
         m_fLeft += 65536.0;
      }
      if (m_fRight >= 32768.0) {
         m_fRight -= 65536.0;
      } else if (m_fRight < -32768.0) {
         m_fRight += 65536.0;
      }
      m_tReadings.EncoderLeftWheel = int(floor(m_fLeft));
      m_tReadings.EncoderRightWheel = int(floor(m_fRight));
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderDefaultSensor::Reset() {
      m_fLeft = 0.0;
      m_fRight = 0.0;
      m_tReadings.EncoderLeftWheel = 0;
      m_tReadings.EncoderRightWheel = 0;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuck2EncoderDefaultSensor,
                   "epuck2_encoder", "default",
                   "Daniel H. Stolfi based on Carlo Pinciroli's work",
                   "1.0",
                   "The EPuck2 wheel encoder sensor.",

                   "This sensor provides the value of the wheels encoders\n"
                   "between -32768 and 32768, where 1000 steps represent a complete wheel revolution.\n"
                   "In controllers, you must include the ci_epuck2_encoder_sensor.h header.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_encoder implementation=\"default\" />\n"
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
                   "reading. The final sensor reading is always normalised in the [-32768-32768] range.\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_encoder implementation=\"rot_z_only\"\n"
                   "                        noise_level=\"1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",

                   "Usable"
		  );

}
