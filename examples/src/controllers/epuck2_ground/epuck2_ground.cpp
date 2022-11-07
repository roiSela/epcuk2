/**
 * @file <epuck2_ground.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_ground.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

/****************************************/
/****************************************/

CCI_EPuck2Ground::CCI_EPuck2Ground() :
   m_pcWheels(NULL),
   m_pcGroundSensor(NULL),
   m_bVerb(false),
   m_fWheelVelocityLeft(2.5f),
   m_fWheelVelocityRight(2.5f) {}

/****************************************/
/****************************************/

void CCI_EPuck2Ground::Init(TConfigurationNode& t_node) {
   m_pcWheels        = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcGroundSensor  = GetSensor  <CCI_Epuck2GroundSensor          >("epuck2_ground"        );

   std::string sLog = "";
   GetNodeAttributeOrDefault(t_node, "left", m_fWheelVelocityLeft, m_fWheelVelocityLeft);
   GetNodeAttributeOrDefault(t_node, "right", m_fWheelVelocityRight, m_fWheelVelocityRight);
   GetNodeAttributeOrDefault(t_node, "verbose", m_bVerb, m_bVerb);
   GetNodeAttributeOrDefault(t_node, "log", sLog, sLog);
   if (!sLog.empty()) {
      m_cLogFile.open(CCI_Controller::GetId() + "_" + sLog, std::ofstream::out | std::ofstream::trunc);
      m_cLogFile << "Tick,Ground1,Ground2,Ground3" << std::endl;
   }
}

/****************************************/
/****************************************/

void CCI_EPuck2Ground::ControlStep() {

   unsigned uTick = CSimulator::GetInstance().GetSpace().GetSimulationClock();
   std::string sId = CCI_Controller::GetId();

   /* Get readings from ground sensor */
   const std::vector<SInt32>& tReadings = m_pcGroundSensor->GetReadings();
   if (m_bVerb) {
      LOG << "Ground: ";
      for(size_t i = 0; i < tReadings.size(); ++i) {
         LOG << tReadings[i] << " # ";
      }
      LOG << std::endl;
   }

   if (m_cLogFile.is_open()) {
      m_cLogFile << uTick << "," << std::setprecision(6);
      m_cLogFile << tReadings[0] << "," << tReadings[1] << "," << tReadings[2] << std::endl;
   }

   m_pcWheels->SetLinearVelocity(m_fWheelVelocityLeft, m_fWheelVelocityRight);

}

/****************************************/
/****************************************/

void CCI_EPuck2Ground::Destroy() {
   if (m_cLogFile.is_open()) {
      m_cLogFile.close();
   }
}

/****************************************/
/****************************************/


REGISTER_CONTROLLER(CCI_EPuck2Ground, "epuck2_ground_controller")
