/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_tof_default_sensor.cpp>
 *
 * @author Daniel H. Stolfi based on the Danesh Tarapore's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include "epuck2_tof_equipped_entity.h"
#include "epuck2_tof_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static CRange<SInt32> UNIT(20, 2000);

   /****************************************/
   /****************************************/

   CEPuck2TOFDefaultSensor::CEPuck2TOFDefaultSensor() :
      m_pcEmbodiedEntity(NULL),
      m_pcTOFEntity(NULL),
      m_pcControllableEntity(NULL),
      m_bShowRays(false),
      m_pcRNG(NULL),
      m_bAddNoise(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()) {}

   /****************************************/
   /****************************************/

   void CEPuck2TOFDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      try {
         m_pcEmbodiedEntity = &(c_entity.GetComponent<CEmbodiedEntity>("body"));
         m_pcControllableEntity = &(c_entity.GetComponent<CControllableEntity>("controller"));
         m_pcTOFEntity = &(c_entity.GetComponent<CEPuck2TOFEquippedEntity>("tof_sensor"));
         m_pcTOFEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Can't set robot for the TOF default sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_EPuck2TOFSensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise level */
         Real fNoiseLevel = 0.0f;
         GetNodeAttributeOrDefault(t_tree, "noise_level", fNoiseLevel, fNoiseLevel);
         if(fNoiseLevel < 0.0f) {
            THROW_ARGOSEXCEPTION("Can't specify a negative value for the noise level of the TOF sensor");
         }
         else if(fNoiseLevel > 0.0f) {
            m_bAddNoise = true;
            m_cNoiseRange.Set(-fNoiseLevel, fNoiseLevel);
            m_pcRNG = CRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in default TOF sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFDefaultSensor::Update()
   {
      /* Ray used for scanning the environment for obstacles */
      CRay3 cScanningRay;
      CVector3 cRayStart, cRayEnd;
      /* Buffers to contain data about the intersection */
      SEmbodiedEntityIntersectionItem sIntersection;
      /* Compute ray for sensor i */
      cRayStart = m_pcTOFEntity->GetSensor(0).Offset;
      cRayStart.Rotate(m_pcTOFEntity->GetSensor(0).Anchor.Orientation);
      cRayStart += m_pcTOFEntity->GetSensor(0).Anchor.Position;
      cRayEnd = m_pcTOFEntity->GetSensor(0).Offset;
      cRayEnd += m_pcTOFEntity->GetSensor(0).Direction;
      cRayEnd.Rotate(m_pcTOFEntity->GetSensor(0).Anchor.Orientation);
      cRayEnd += m_pcTOFEntity->GetSensor(0).Anchor.Position;
      cScanningRay.Set(cRayStart,cRayEnd);
      /* Compute reading */
      Real fReading = 2.0f; /* No intersection */
      /* Get the closest intersection */
      if(GetClosestEmbodiedEntityIntersectedByRay(sIntersection,
                                                   cScanningRay,
                                                   *m_pcEmbodiedEntity)) {
         /* There is an intersection */
         if(m_bShowRays) {
            m_pcControllableEntity->AddIntersectionPoint(cScanningRay,
                                                         sIntersection.TOnRay);
            m_pcControllableEntity->AddCheckedRay(true, cScanningRay);
         }
         fReading = cScanningRay.GetDistance(sIntersection.TOnRay);
      } else {

         if(m_bShowRays) {
            m_pcControllableEntity->AddCheckedRay(false, cScanningRay);
         }
      }
      /* Apply noise to the sensor */
      if(m_bAddNoise) {
         fReading += m_pcRNG->Uniform(m_cNoiseRange) * 2.0f;
      }
      m_iReading = int(round(fReading * 1000.0f));
      /* Trunc the reading between 0.0 and 2000.0 */
      UNIT.TruncValue(m_iReading);
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFDefaultSensor::Reset() {
      m_iReading = 2000;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEPuck2TOFDefaultSensor,
                   "epuck2_tof", "default",
                   "Daniel H. Stolfi based on Danesh Tarapore's work",
                   "1.0",
                   "The EPuck2 ToF (Time of Fight) sensor.",
                   "This sensor gets the distance to a possible obstacle in front of the robot. The return value\n"
                   "is between 20 and 2000, where 20 is the minimum measured distance and 2000 means that no object\n"
                   "is detected in 2 metres range.\n"
                   "Values between 10 and 2000 are the distance in millimetres.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_tof implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the ray shot by the proximity sensor in the OpenGL\n"
                   "visualisation. This can be useful for sensor debugging but also to understand\n"
                   "what's wrong in your controller. In OpenGL, the ray is drawn in cyan when\n"
                   "it is not obstructed and in purple when it is. In case the ray is\n"
                   "obstructed, a black dot is drawn where the intersection occurred.\n"
                   "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
                   "example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_tof implementation=\"default\"\n"
                   "                    show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "It is possible to add uniform noise to the sensor, thus matching the\n"
                   "characteristics of a real robot better. This can be done with the attribute\n"
                   "\"noise_level\", whose allowed range is in [-1,1] and is added to the calculated\n"
                   "reading. The final sensor reading is always normalised in the [10-2000] range.\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <epuck2_tof implementation=\"default\"\n"
                   "                    noise_level=\"0.1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",
                   "Usable"
		  );

}
