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
         /* Get full battery charge */
         //GetNodeAttributeOrDefault(t_tree, "full_charge",   m_fFullCharge,       m_fFullCharge);
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
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"time_motion\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   Real CEPuck2BatteryDischargeModelCubic::FindX(const Real f_charge) {
      Real f_error = 9e9;
      int i = 0;
      Real f_min = 0;
      Real f_max = MAX_X;
      Real f_mid = 0;
      while (f_error > 0.000001 && i < 50) {
         f_mid = f_min + (f_max - f_min) / 2.0;
         f_error = f_charge - Eval(Px, f_mid);
         if (f_error < 0) {
            f_min = f_mid;
            f_error = -f_error;
         } else {
            f_max = f_mid;
         }
         i++;
      }
      return f_mid;
   }

   Real CEPuck2BatteryDischargeModelCubic::Delta(const Real f_charge, const Real f_delta) {

      Real x = FindX(f_charge);
      return Eval(Px, x + f_delta);
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

         Real f_delta_d = DELTA_D3;
         if (m_pcBattery->GetAvailableCharge() > TH1) {
            f_delta_d = DELTA_D1;
         } else if (m_pcBattery->GetAvailableCharge() > TH2) {
            f_delta_d = DELTA_D2;
         }
         m_pcBattery->SetAvailableCharge(
            Max<Real>(
               0.0,
               m_pcBattery->GetAvailableCharge() -
               DELTA_S * CSimulator::GetInstance().GetPhysicsEngines()[0]->GetSimulationClockTick() -
               f_delta_d * fDeltaPos));

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
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"time_motion\"", ex);
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
         Real fSpeed = fDeltaPos / fDeltaT;
         int spdL = floor(fSpeed);
         int spdH = ceil(fSpeed);

         if (spdL == spdH) {
            // matching function
         } else {
            // interpolation required
         }

         /* Calculate new level */
         m_pcBattery->SetAvailableCharge(
            Max<Real>(
               0.0,
               m_pcBattery->GetAvailableCharge() -
               m_fDelta -
               m_fPosFactor * fDeltaPos));
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
         THROW_ARGOSEXCEPTION_NESTED("While setting body for battery model \"time_motion\"", ex);
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
         m_pcBattery->SetAvailableCharge(
            Max<Real>(
               0.0,
               m_pcBattery->GetAvailableCharge() -
               m_fDelta -
               m_fPosFactor * fDeltaPos));
         /* Save position for next step */
         m_cOldPosition = m_psAnchor->Position;
         m_cOldOrientation = m_psAnchor->Orientation;
      }
   }

   
   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2BatteryEquippedEntity);
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelCubic, "cubic");
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelLinear, "linear");
   REGISTER_BATTERY_DISCHARGE_MODEL(CEPuck2BatteryDischargeModelSimple, "simple");

   /****************************************/
   /****************************************/

}
