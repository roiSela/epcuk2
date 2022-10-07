/**
 * @file <argos3/plugins/robots/e-puck2/battery_equipped_entity.cpp>
 *
 * @author  Daniel H. Stolfi based on the Adhavan Jayabalan's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */
#include "epuck2_battery_equipped_entity.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CEPuck2BatteryEquippedEntity::CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent) :
      CEntity(pc_parent),
      m_fFullCharge(1.0),
      m_fAvailableCharge(m_fFullCharge),
      m_pcDischargeModel(nullptr) {
      SetDischargeModel(new CEPuck2BatteryDischargeModelSimple());
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2BatteryEquippedEntity::CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent,
                                                  const std::string &str_id,
                                                  CEPuck2BatteryDischargeModel* pc_discharge_model,
                                                  Real f_start_charge,
                                                  Real f_full_charge) :
      CEntity(pc_parent, str_id),
      m_fFullCharge(f_full_charge),
      m_fAvailableCharge(f_start_charge),
      m_pcDischargeModel(nullptr) {
      SetDischargeModel(pc_discharge_model);
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2BatteryEquippedEntity::CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent,
                                                  const std::string& str_id,
                                                  const std::string& str_discharge_model,
                                                  Real f_start_charge,
                                                  Real f_full_charge) :
      CEntity(pc_parent, str_id),
      m_fFullCharge(f_full_charge),
      m_fAvailableCharge(f_start_charge),
      m_pcDischargeModel(nullptr) {
      SetDischargeModel(str_discharge_model);
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2BatteryEquippedEntity::~CEPuck2BatteryEquippedEntity() {
      /* Get rid of battery discharge model */
      delete m_pcDischargeModel;
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         CEntity::Init(t_tree);
         /* Get initial battery level */
         std::string strDischargeModel = "simple";
         GetNodeAttributeOrDefault(t_tree, "discharge_model", strDischargeModel, strDischargeModel);
         SetDischargeModel(strDischargeModel);
         m_pcDischargeModel->Init(t_tree);
         /* Get initial battery charge */
         GetNodeAttributeOrDefault(t_tree, "start_charge",  m_fAvailableCharge,  m_fAvailableCharge);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the battery sensor equipped entity \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryEquippedEntity::Update() {
      if(CSimulator::GetInstance().GetSpace().GetSimulationClock() > 0) {
         if(m_pcDischargeModel)
            /* Call the discharge model */
            (*m_pcDischargeModel)();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryEquippedEntity::SetDischargeModel(CEPuck2BatteryDischargeModel* pc_model) {
      if(m_pcDischargeModel) delete m_pcDischargeModel;
      m_pcDischargeModel = pc_model;
      if(m_pcDischargeModel)
         m_pcDischargeModel->SetBattery(this);
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryEquippedEntity::SetDischargeModel(const std::string& str_model) {
      if(m_pcDischargeModel) delete m_pcDischargeModel;
      if(str_model != "") {
         m_pcDischargeModel = TFactoryBatteryDischargeModel::New(str_model);
         m_pcDischargeModel->SetBattery(this);
      }
   }

   /****************************************/
   /****************************************/

   CEPuck2BatteryDischargeModel::CEPuck2BatteryDischargeModel() :
      m_pcBattery(nullptr) {
   }

   /****************************************/
   /****************************************/

   CEPuck2BatteryDischargeModel::~CEPuck2BatteryDischargeModel() {
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModel::SetBattery(CEPuck2BatteryEquippedEntity* pc_battery) {
      m_pcBattery = pc_battery;
   }

   /****************************************/
   /****************************************/


   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelCubic::Init(TConfigurationNode& t_tree) {
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelCubic::SetBattery(CEPuck2BatteryEquippedEntity* pc_battery) {
      try {
         /* Execute default logic */
         CEPuck2BatteryDischargeModel::SetBattery(pc_battery);
         /* Get a hold of the body and anchor of the entity that contains the battery */
         CEntity* pcRoot = &pc_battery->GetRootEntity();
         auto* cComp = dynamic_cast<CComposableEntity*>(pcRoot);
         if(cComp != nullptr) {
            auto& cBody = cComp->GetComponent<CEmbodiedEntity>("body");
            m_psAnchor = &cBody.GetOriginAnchor();
            m_cOldPosition = m_psAnchor->Position;
         }
         else {
            THROW_ARGOSEXCEPTION("Root entity is not composable");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"cubic\"", ex);
      }
   }

   Real CEPuck2BatteryDischargeModelCubic::CubicRoot(const Real a, const Real b, const Real c, const Real d, const Real y) {
     Real new_d = d - y;
     Real d0 = b*b - 3*a*c;
     Real d1 = 2*b*b*b - 9*a*b*c + 27*a*a*new_d;
     Real C = std::cbrt((d1 + sqrt(d1*d1 - 4*d0*d0*d0)) / 2);
     return (-1/(3*a) * (b + C + d0/C));
   }


   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelCubic::operator()() {
      if(m_pcBattery->GetAvailableCharge() > 0.0) {
         /* Calculate delta position */
         Real fDeltaPos = Distance(m_psAnchor->Position,
                                   m_cOldPosition);
         /* Calculate delta orientation */
         CQuaternion cDeltaOrient =
            m_cOldOrientation.Inverse() *
            m_psAnchor->Orientation;
         CRadians cDeltaAngle;
         CVector3 cDeltaAxis;
         cDeltaOrient.ToAngleAxis(cDeltaAngle, cDeltaAxis);
         /* Calculate new level */
         if (fDeltaPos == 0.0) {
            if (cDeltaAngle.GetValue() != 0.0) {
               fDeltaPos = cDeltaAngle.GetValue() * 0.0265f;
            }
         }
         Real fDeltaT = CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick();
         Real fSpeed = std::round(std::max(0.0, std::min(0.150, fDeltaPos / fDeltaT)) * 10000.0) / 100.0; // cm/s
         Real fBat = m_pcBattery->GetAvailableCharge();

         // First interpolation point: LOW
         int spdL = std::min(15.0, floor(fSpeed));
         Real a1 = Q[spdL][3];
         Real b1 = Q[spdL][2];
         Real c1 = Q[spdL][1];
         Real d1 = Q[spdL][0];
         Real e1 = Q[spdL][4];
         Real x1 = std::max(0.0, CubicRoot(a1, b1, c1, d1, fBat));
         Real y1;
         if (x1 >= e1) {
            y1 = 0.0;
         } else {
            x1 += fDeltaT;
            y1 = a1*x1*x1*x1 + b1*x1*x1 + c1*x1 + d1;
         }

         int spdH = std::max(0.0, ceil(fSpeed));

         if (spdL == spdH) {
            m_pcBattery->SetAvailableCharge(Max<Real>(0.0, y1));
         } else {
            // Second interpolation point
            Real a2 = Q[spdH][3];
            Real b2 = Q[spdH][2];
            Real c2 = Q[spdH][1];
            Real d2 = Q[spdH][0];
            Real e2 = Q[spdH][4];
            Real x2 = std::max(0.0, CubicRoot(a2, b2, c2, d2, fBat));
            Real y2;
            if (x2 >= e2) {
               y2 = 0.0;
            } else {
               x2 += fDeltaT;
               y2 = a2*x2*x2*x2 + b2*x2*x2 + c2*x2 + d2;
            }

            Real d = (fSpeed - spdL) / (spdH - spdL);
            if (y1 < y2) {
               m_pcBattery->SetAvailableCharge(Max<Real>(0.0, y1 + (y2 - y1) * d));
            } else {
               m_pcBattery->SetAvailableCharge(Max<Real>(0.0, y2 + (y1 - y2) * d));
            }
         }

         /* Save position for next step */
         m_cOldPosition = m_psAnchor->Position;
         m_cOldOrientation = m_psAnchor->Orientation;
      }
   }

   /****************************************/
   /****************************************/

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelApprox::Init(TConfigurationNode& t_tree) {
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelApprox::SetBattery(CEPuck2BatteryEquippedEntity* pc_battery) {
      try {
         /* Execute default logic */
         CEPuck2BatteryDischargeModel::SetBattery(pc_battery);
         /* Get a hold of the body and anchor of the entity that contains the battery */
         CEntity* pcRoot = &pc_battery->GetRootEntity();
         auto* cComp = dynamic_cast<CComposableEntity*>(pcRoot);
         if(cComp != nullptr) {
            auto& cBody = cComp->GetComponent<CEmbodiedEntity>("body");
            m_psAnchor = &cBody.GetOriginAnchor();
            m_cOldPosition = m_psAnchor->Position;
         }
         else {
            THROW_ARGOSEXCEPTION("Root entity is not composable");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"approx\"", ex);
      }
   }


   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelApprox::operator()() {
      if(m_pcBattery->GetAvailableCharge() > 0.0) {
         /* Calculate delta position */
         Real fDeltaPos = Distance(m_psAnchor->Position,
                                   m_cOldPosition);
         /* Calculate delta orientation */
         CQuaternion cDeltaOrient =
            m_cOldOrientation.Inverse() *
            m_psAnchor->Orientation;
         CRadians cDeltaAngle;
         CVector3 cDeltaAxis;
         cDeltaOrient.ToAngleAxis(cDeltaAngle, cDeltaAxis);
         /* Calculate new level */
         if (fDeltaPos == 0.0) {
            if (cDeltaAngle.GetValue() != 0.0) {
               fDeltaPos = cDeltaAngle.GetValue() * 0.0265f;
            }
         }
         Real fDeltaT = CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick();
         Real fSpeed = std::round(std::max(0.0, std::min(0.150, fDeltaPos / fDeltaT)) * 10000.0) / 100.0; // cm/s
         Real fBat = m_pcBattery->GetAvailableCharge();

         int spdL = std::max(0.0, floor(fSpeed));
         int spdH = std::min(15.0, ceil(fSpeed));

         // First interpolation point: LOW
         Real m1, h1;
         if (fBat <= M1[spdL][4] && fBat >= M1[spdL][5]) {
            h1 = M1[spdL][0];
            m1 = M1[spdL][1];
         } else if (fBat <= M2[spdL][4] && fBat >= M2[spdL][5]) {
            h1 = M2[spdL][0];
            m1 = M2[spdL][1];
         } else if (fBat <= M3[spdL][4] && fBat >= M3[spdL][5]) {
            h1 = M3[spdL][0];
            m1 = M3[spdL][1];
         } else {
            h1 = M4[spdL][0];
            m1 = M4[spdL][1];
         }

         Real x1 = (fBat - h1) / m1;
         Real y1 = m1 * (x1 + fDeltaT) + h1;

         if (spdL == spdH) {
            m_pcBattery->SetAvailableCharge(Max<Real>(0.0, y1));
         } else {
            // Second interpolation point: RIGHT
            Real m2, h2;
            if (fBat <= M1[spdH][4] && fBat >= M1[spdH][5]) {
               h2 = M1[spdH][0];
               m2 = M1[spdH][1];
            } else if (fBat <= M2[spdH][4] && fBat >= M2[spdH][5]) {
               h2 = M2[spdH][0];
               m2 = M2[spdH][1];
            } else if (fBat <= M3[spdH][4] && fBat >= M3[spdH][5]) {
               h2 = M3[spdH][0];
               m2 = M3[spdH][1];
            } else {
               h2 = M4[spdH][0];
               m2 = M4[spdH][1];
            }

            Real x2 = (fBat - h2) / m2;
            Real y2 = m2 * (x2 + fDeltaT) + h2;

            Real d = (fSpeed - spdL) / (spdH - spdL);
            if (y1 < y2) {
               m_pcBattery->SetAvailableCharge(Max<Real>(0.0, y1 + (y2 - y1) * d));
            } else {
               m_pcBattery->SetAvailableCharge(Max<Real>(0.0, y2 + (y1 - y2) * d));
            }
         }
         /* Save position for next step */
         m_cOldPosition = m_psAnchor->Position;
         m_cOldOrientation = m_psAnchor->Orientation;
      }
   }

   /****************************************/
   /****************************************/

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelLinear::Init(TConfigurationNode& t_tree) {
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelLinear::SetBattery(CEPuck2BatteryEquippedEntity* pc_battery) {
      try {
         /* Execute default logic */
         CEPuck2BatteryDischargeModel::SetBattery(pc_battery);
         /* Get a hold of the body and anchor of the entity that contains the battery */
         CEntity* pcRoot = &pc_battery->GetRootEntity();
         auto* cComp = dynamic_cast<CComposableEntity*>(pcRoot);
         if(cComp != nullptr) {
            auto& cBody = cComp->GetComponent<CEmbodiedEntity>("body");
            m_psAnchor = &cBody.GetOriginAnchor();
            m_cOldPosition = m_psAnchor->Position;
         }
         else {
            THROW_ARGOSEXCEPTION("Root entity is not composable");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"linear\"", ex);
      }
   }


   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelLinear::operator()() {
      if(m_pcBattery->GetAvailableCharge() > 0.0) {
         /* Calculate delta position */
         Real fDeltaPos = Distance(m_psAnchor->Position,
                                   m_cOldPosition);
         /* Calculate delta orientation */
         CQuaternion cDeltaOrient =
            m_cOldOrientation.Inverse() *
            m_psAnchor->Orientation;
         CRadians cDeltaAngle;
         CVector3 cDeltaAxis;
         cDeltaOrient.ToAngleAxis(cDeltaAngle, cDeltaAxis);
         /* Calculate new level */
         if (fDeltaPos == 0.0) {
            if (cDeltaAngle.GetValue() != 0.0) {
               fDeltaPos = cDeltaAngle.GetValue() * 0.0265f;
            }
         }
         Real fDeltaT = CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick();
         Real fSpeed = std::round(std::max(0.0, std::min(0.150, fDeltaPos / fDeltaT)) * 10000.0) / 100.0; // cm/s
         int spdL = std::max(0.0, floor(fSpeed));
         int spdH = std::min(15.0, ceil(fSpeed));

         if (spdL == spdH) {
            // matching function
            float m = L[spdL];
            m_pcBattery->SetAvailableCharge(
               Max<Real>(
                     0.0,
                     m_pcBattery->GetAvailableCharge() +
                     m * fDeltaT));
         } else {
            // interpolation required
            float mL = L[spdL];
            float mH = L[spdH];
            float yL = m_pcBattery->GetAvailableCharge() + mL * fDeltaT;
            float yH = m_pcBattery->GetAvailableCharge() + mH * fDeltaT;
            float d = (fSpeed - spdL) / (spdH - spdL);

            if (yH < yL) {
               m_pcBattery->SetAvailableCharge(
                  Max<Real>(
                        0.0,
                        yH + (yL - yH) * d));
            } else {
               m_pcBattery->SetAvailableCharge(
                  Max<Real>(
                        0.0,
                        yL + (yH - yL) * d));
            }
         }
         /* Save position for next step */
         m_cOldPosition = m_psAnchor->Position;
         m_cOldOrientation = m_psAnchor->Orientation;
      }
   }

   /****************************************/
   /****************************************/

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelSimple::Init(TConfigurationNode& t_tree) {
   }

   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelSimple::SetBattery(CEPuck2BatteryEquippedEntity* pc_battery) {
      try {
         /* Execute default logic */
         CEPuck2BatteryDischargeModel::SetBattery(pc_battery);
         /* Get a hold of the body and anchor of the entity that contains the battery */
         CEntity* pcRoot = &pc_battery->GetRootEntity();
         auto* cComp = dynamic_cast<CComposableEntity*>(pcRoot);
         if(cComp != nullptr) {
            auto& cBody = cComp->GetComponent<CEmbodiedEntity>("body");
            m_psAnchor = &cBody.GetOriginAnchor();
            m_cOldPosition = m_psAnchor->Position;
         }
         else {
            THROW_ARGOSEXCEPTION("Root entity is not composable");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"simple\"", ex);
      }
   }


   /****************************************/
   /****************************************/

   void CEPuck2BatteryDischargeModelSimple::operator()() {
      if(m_pcBattery->GetAvailableCharge() > 0.0) {
         /* Calculate delta position */
         Real fDeltaPos = Distance(m_psAnchor->Position,
                                   m_cOldPosition);
         /* Calculate delta orientation */
         CQuaternion cDeltaOrient =
            m_cOldOrientation.Inverse() *
            m_psAnchor->Orientation;
         CRadians cDeltaAngle;
         CVector3 cDeltaAxis;
         cDeltaOrient.ToAngleAxis(cDeltaAngle, cDeltaAxis);
         /* Calculate new level */
         if (fDeltaPos == 0.0) {
            if (cDeltaAngle.GetValue() != 0.0) {
               fDeltaPos = cDeltaAngle.GetValue() * 0.0265f;
            }
         }
         /* Calculate new level */
         if (fDeltaPos == 0) {
            m_pcBattery->SetAvailableCharge(
               Max<Real>(
                  0.0,
                  m_pcBattery->GetAvailableCharge() +
                  M0 * CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick()));
         } else {
            m_pcBattery->SetAvailableCharge(
               Max<Real>(
                  0.0,
                  m_pcBattery->GetAvailableCharge() +
                  M1 * CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick()));
         }
         /* Save position for next step */
         m_cOldPosition = m_psAnchor->Position;
         m_cOldOrientation = m_psAnchor->Orientation;
      }
   }

   
   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2BatteryEquippedEntity);
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelCubic, "cubic");
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelApprox, "approx");
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelLinear, "linear");
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelSimple, "simple");

   /****************************************/
   /****************************************/

}
