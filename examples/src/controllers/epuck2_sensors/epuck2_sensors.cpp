/**
 * @file <epuck2_sensors.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_sensors.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>

/****************************************/
/****************************************/

CEPuck2Sensors::CEPuck2Sensors() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABAct(NULL),
   m_pcRABSens(NULL),
   m_pcLedAct(NULL),
   m_pcLightSens(NULL),
   m_pcTOFSensor(NULL),
   m_pcEncoderSensor(NULL),
   m_pcGroundSensor(NULL),
   m_pcCamera(NULL),
   m_pcBattery(NULL),
   m_fWheelVelocity(2.5f),
   m_uLED(0),
   m_bGreen(false) {}

/****************************************/
/****************************************/

void CEPuck2Sensors::Init(TConfigurationNode& t_node) {
   m_pcWheels        = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity     = GetSensor  <CCI_EPuck2ProximitySensor       >("epuck2_proximity"     );
   m_pcRABAct        = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABSens       = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_pcLedAct        = GetActuator<CCI_EPuck2LEDsActuator          >("epuck2_leds"          );
   m_pcLightSens     = GetSensor  <CCI_LightSensor                 >("epuck2_light"         );
   m_pcCamera        = GetSensor  <CCI_ColoredBlobPerspectiveCameraSensor>("epuck2_colored_blob_perspective_camera");
   m_pcTOFSensor     = GetSensor  <CCI_EPuck2TOFSensor             >("epuck2_tof"           );
   m_pcGroundSensor  = GetSensor  <CCI_GroundSensor                >("epuck2_ground"        );
   m_pcEncoderSensor = GetSensor  <CCI_EPuck2EncoderSensor         >("epuck2_encoder"       );
   m_pcBattery       = GetSensor  <CCI_BatterySensor               >("epuck2_battery"       );

   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   if (CCI_Controller::GetId()=="1") {
         m_pcCamera->Enable();
   }

   m_pcLedAct->SetAllBlack();
}

/****************************************/
/****************************************/

void CEPuck2Sensors::ControlStep() {

   unsigned uTick = (&CSimulator::GetInstance())->GetSpace().GetSimulationClock();
   std::string sId = CCI_Controller::GetId();
   char chMsg = '-';
   CByteArray cData = CByteArray();
   LOG << sId << std::fixed << std::setprecision(1) << std::endl;


   /* Get RAB messages */
   const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
   if(!tMsgs.empty()) {

      for(size_t i = 0; i < tMsgs.size(); ++i) {
         CByteArray rx = tMsgs[i].Data;
         LOG << "Msg:" << rx << "  Bearing:" << tMsgs[i].HorizontalBearing.GetValue() << "," << tMsgs[i].VerticalBearing.GetValue() << "  Range:" << tMsgs[i].Range << std::endl;;
         chMsg = rx[0];
      }
   }

   /* Get readings from light sensor */
   const std::vector<Real>& tLightReads = m_pcLightSens->GetReadings();
   LOG << "Light: " << std::setprecision(1);
   for(size_t i = 0; i < tLightReads.size(); ++i) {
      LOG << tLightReads[i] << " # ";
   }
   LOG << std::endl;


   /* Get readings from proximity sensor */
   const CCI_EPuck2ProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   LOG << "Proximity: " << std::setprecision(1);
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      LOG << tProxReads[i].Value << " # ";
   }
   LOG << std::endl;

   /* Get readings from ToF sensor */
   LOG << "TOF: " << std::setprecision(1) << m_pcTOFSensor->GetReadings() << std::endl;

   /* Get readings from ground sensor */
   const std::vector<Real>& tReadings = m_pcGroundSensor->GetReadings();
   LOG << "Ground: ";
   for(size_t i = 0; i < tReadings.size(); ++i) {
      LOG << tReadings[i] << " # ";
   }
   LOG << std::endl;

   /* Get readings from encoder sensor */
   const CCI_EPuck2EncoderSensor::SReading& tEncoderReads = m_pcEncoderSensor->GetReadings();
   LOG << "Encoders: " << tEncoderReads.EncoderLeftWheel << " # " << tEncoderReads.EncoderRightWheel << std::endl;

   /* Perspective Camera */
   const CCI_ColoredBlobPerspectiveCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   LOG << "Camera: " << std::endl;
   for (size_t i = 0; i < sReadings.BlobList.size(); i++) {
       CCI_ColoredBlobPerspectiveCameraSensor::SBlob* sBlob = sReadings.BlobList[i];
      LOG << "...(Color = " << sBlob->Color << ", X = " << sBlob->X << ",Y = " << sBlob->Y << ")" << std::endl;
   }

   /* Get readings from Battery sensor */
   CCI_BatterySensor::SReading tBatReading = m_pcBattery->GetReading();
   LOG << "Battery - Available Charge: " << std::fixed << std::setprecision(3) << tBatReading.AvailableCharge << "  Time Left: " << tBatReading.TimeLeft << std::endl;

   /* Movement */
   bool bFront = m_pcTOFSensor->GetReadings() <= 20.0;
   cData = CByteArray();

   if (sId == "1") {
      if (chMsg == '2') {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
         cData << (UInt8) '3';
      } else {
         if (bFront) {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            cData << (UInt8) '1';
         } else {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            cData << (UInt8) '-';
         }
      }
   }

   if (sId == "2") {
      if (chMsg == '1') {
         if (bFront) {
            m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            cData << (UInt8) '2';
         } else {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            cData << (UInt8) '-';
         }
      } else {
         if (chMsg == '3') {
            cData << (UInt8) '2';
            m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
         } else {
            cData << (UInt8) '-';
         }
      }
   }

   if (uTick % 2 == 0) {
       m_uLED++;
       m_pcLedAct->SetFrontLed(m_uLED % 2 == 0);
       if (sId=="2") {
          m_pcLedAct->SetBodyLed(m_bGreen);
       }
       m_pcLedAct->SetRedLed1(m_uLED > 2);
       m_pcLedAct->SetRedLed3(m_uLED > 4);
       m_pcLedAct->SetRedLed5(m_uLED > 6);
       m_pcLedAct->SetRedLed7(m_uLED > 8);
       if (m_uLED > 10) {
           m_uLED = 0;
           m_bGreen = ! m_bGreen;
       }
       unsigned uCte = m_uLED * 25;
       m_pcLedAct->SetRGBLed2Color(CColor(uCte, 0, uCte));
       m_pcLedAct->SetRGBLed4Color(CColor(0, uCte, 0));
       m_pcLedAct->SetRGBLed6Color(CColor(0, 0, uCte));
       m_pcLedAct->SetRGBLed8Color(CColor(uCte, uCte, uCte));

   }

   m_pcRABAct->SetData(cData);

}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEPuck2Sensors, "epuck2_sensors_controller")
