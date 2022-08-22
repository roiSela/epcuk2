/**
 * @file <epuck2_camera.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_camera.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

/****************************************/
/****************************************/

CEPuck2Camera::CEPuck2Camera() :
   m_pcWheels(NULL),
   m_pcCamera(NULL),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CEPuck2Camera::Init(TConfigurationNode& t_node) {
   m_pcWheels        = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcCamera        = GetSensor  <CCI_ColoredBlobPerspectiveCameraSensor>("epuck2_colored_blob_perspective_camera");

   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   m_pcCamera->Enable();

}

/****************************************/
/****************************************/

void CEPuck2Camera::ControlStep() {

   unsigned uTick = (&CSimulator::GetInstance())->GetSpace().GetSimulationClock();
   std::string sId = CCI_Controller::GetId();
   // LOG << sId << std::fixed << std::setprecision(1) << std::endl;

   CRadians cZAngle, cYAngle, cXAngle;
   CEPuck2Entity cEPuck =
         *dynamic_cast<CEPuck2Entity*>(&(&CSimulator::GetInstance())->GetSpace().GetEntity(
               CCI_Controller::GetId()));
   cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   LOG << "Angle: " << ToDegrees(cZAngle).GetValue() << std::endl;

   /* Perspective Camera */
   const CCI_ColoredBlobPerspectiveCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   LOG << "Camera: " << std::endl;
   for (size_t i = 0; i < sReadings.BlobList.size(); i++) {
       CCI_ColoredBlobPerspectiveCameraSensor::SBlob* sBlob = sReadings.BlobList[i];
      LOG << "...(Color = " << sBlob->Color << ", X = " << sBlob->X << ",Y = " << sBlob->Y << ")" << std::endl;
   }

   /* Movement */
   m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);

}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEPuck2Camera, "epuck2_camera_controller")
