/**
 * @file <epuck2_encoders.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_encoders.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

/****************************************/
/****************************************/

CCI_EPuck2Encoders::CCI_EPuck2Encoders() :
   m_pcWheels(NULL),
   m_pcEncoderSensor(NULL),
   m_pcTOFSensor(NULL),
   m_iPreviousEncoder(32768),
   m_fDistance(0.0),
   m_fToFbegin(0.0),
   m_fWheelVelocityLeft(2.5f),
   m_fWheelVelocityRight(2.5f) {}

/****************************************/
/****************************************/

void CCI_EPuck2Encoders::Init(TConfigurationNode& t_node) {
   m_pcWheels        = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcEncoderSensor = GetSensor  <CCI_EPuck2EncoderSensor         >("epuck2_encoder"       );
   m_pcTOFSensor     = GetSensor  <CCI_EPuck2TOFSensor             >("epuck2_tof"           );

   std::string sLog = "";
   GetNodeAttributeOrDefault(t_node, "left", m_fWheelVelocityLeft, m_fWheelVelocityLeft);
   GetNodeAttributeOrDefault(t_node, "right", m_fWheelVelocityRight, m_fWheelVelocityRight);
}

/****************************************/
/****************************************/

void CCI_EPuck2Encoders::ControlStep() {

   unsigned uTick = CSimulator::GetInstance().GetSpace().GetSimulationClock();
   std::string sId = CCI_Controller::GetId();

   /* Get readings from encoder sensor */
   const CCI_EPuck2EncoderSensor::SReading& tEncoderReads = m_pcEncoderSensor->GetReadings();
   int iDiff = 0;
   int iEnc = tEncoderReads.EncoderLeftWheel + 32768;
   if (iEnc < m_iPreviousEncoder) {
      iDiff = 65535 - m_iPreviousEncoder + iEnc;
   } else {
      iDiff = iEnc - m_iPreviousEncoder;
   }
   m_iPreviousEncoder = iEnc;
   m_fDistance += float(iDiff) * 2 * CRadians::PI.GetValue() * 0.0205f;
   //LOG << tEncoderReads.EncoderLeftWheel << " " << m_iPreviousEncoder << " " << iDiff << " " << m_fDistance << std::endl;
   LOG << "Est. Distance (mm): " << std::fixed << std::setprecision(1) << m_fDistance << std::endl;

   /* Get readings from ToF sensor */
   Real d = m_pcTOFSensor->GetReadings();
   LOG << "TOF: " << d;
   if (uTick == 1) {
      m_fToFbegin = d;
   }
   LOG << " - Distance: " << m_fToFbegin - d << std::endl;

   /* Movement */
   if (d > 20) {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocityLeft, m_fWheelVelocityRight);
   } else {
      CSimulator::GetInstance().Terminate();
   }
}


/****************************************/
/****************************************/

REGISTER_CONTROLLER(CCI_EPuck2Encoders, "epuck2_encoders_controller")
