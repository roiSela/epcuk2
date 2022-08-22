/**
 * @file <epuck2_battery.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_battery.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

/****************************************/
/****************************************/

CEPuck2Battery::CEPuck2Battery() :
   m_pcWheels(NULL),
   m_pcEncoderSensor(NULL),
   m_pcBattery(NULL),
   m_iPreviousEncoder(32768),
   m_fDistance(0.0),
   m_fWheelVelocityLeft(2.5f),
   m_fWheelVelocityRight(2.5f) {}

/****************************************/
/****************************************/

void CEPuck2Battery::Init(TConfigurationNode& t_node) {
   m_pcWheels        = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcEncoderSensor = GetSensor  <CCI_EPuck2EncoderSensor         >("epuck2_encoder"       );
   m_pcBattery       = GetSensor  <CCI_BatterySensor               >("epuck2_battery"       );

   std::string sLog = "";
   GetNodeAttributeOrDefault(t_node, "left", m_fWheelVelocityLeft, m_fWheelVelocityLeft);
   GetNodeAttributeOrDefault(t_node, "right", m_fWheelVelocityRight, m_fWheelVelocityRight);
   GetNodeAttributeOrDefault(t_node, "log", sLog, sLog);
   if (!sLog.empty()) {
      m_cLogFile.open(CCI_Controller::GetId() + "_" + sLog, std::ofstream::out | std::ofstream::trunc);
      m_cLogFile << "Tick,Battery,TimeLeft" << std::endl;
   }
}

/****************************************/
/****************************************/

void CEPuck2Battery::ControlStep() {

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

   /* Get readings from Battery sensor */
   CCI_BatterySensor::SReading tBatReading = m_pcBattery->GetReading();
   LOG << uTick << " - Battery - Available Charge: " << std::fixed << std::setprecision(6) << tBatReading.AvailableCharge << "  Time Left: " << tBatReading.TimeLeft << std::endl;

   if (m_cLogFile.is_open()) {
      m_cLogFile << uTick << "," << tBatReading.AvailableCharge << "," << tBatReading.TimeLeft << std::endl;
   }

   /* Movement */
   if (tBatReading.AvailableCharge > 0.0) {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocityLeft, m_fWheelVelocityRight);

/*      if (uTick < 100) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocityLeft, m_fWheelVelocityRight);
      } else {
         m_pcWheels->SetLinearVelocity(0, 0);
      }*/
   } else {
      CSimulator::GetInstance().Terminate();
   }
}

/****************************************/
/****************************************/

void CEPuck2Battery::Destroy() {
   if (m_cLogFile.is_open()) {
      m_cLogFile.close();
   }
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEPuck2Battery, "epuck2_battery_controller")
