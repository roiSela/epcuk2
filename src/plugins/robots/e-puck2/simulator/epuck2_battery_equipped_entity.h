/**
 * @file <argos3/plugins/robots/e-puck2/battery_equipped_entity.h>
 *
 * @author  Daniel H. Stolfi based on the Adhavan Jayabalan's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_BATTERY_EQUIPPED_ENTITY_H
#define EPUCK2_BATTERY_EQUIPPED_ENTITY_H

namespace argos {
   class CEPuck2BatteryEquippedEntity;
   class CEPuck2BatteryDischargeModel;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <map>

namespace argos {

   /****************************************/
   /****************************************/
   
   /**
    * The battery entity.
    *
    * This entity is used to model the fact that robots have
    * batteries. This entity allows the user to specify the initial
    * level of the battery, its maximum charge, and the discharge
    * model. A few discharge models are offered by default.
    */
   class CEPuck2BatteryEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();

   public:

      CEPuck2BatteryEquippedEntity(CComposableEntity* pc_parent);

      CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent,
                             const std::string& str_id,
                             CEPuck2BatteryDischargeModel* pc_discharge_model = NULL,
                             Real f_start_charge = 1.0,
                             Real f_full_charge = 1.0);

      CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent,
                             const std::string& str_id,
                             const std::string& str_discharge_model,
                             Real f_start_charge = 1.0,
                             Real f_full_charge = 1.0);

      virtual ~CEPuck2BatteryEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "epuck2_battery";
      }

      virtual void Update();

      Real GetFullCharge() const {
         return m_fFullCharge;
      }

      void SetFullCharge(Real f_full_charge) {
         m_fFullCharge = f_full_charge;
      }

      Real GetAvailableCharge() const {
         return m_fAvailableCharge;
      }

      void SetAvailableCharge(Real f_available_charge) {
         m_fAvailableCharge = f_available_charge;
      }

      void SetDischargeModel(CEPuck2BatteryDischargeModel* pc_model);

      void SetDischargeModel(const std::string& str_model);

   protected:

      /** Full charge */
      Real m_fFullCharge;
      /** Available charge */
      Real m_fAvailableCharge;
      /** Discharge model */
      CEPuck2BatteryDischargeModel* m_pcDischargeModel;
   };

   /****************************************/
   /****************************************/
   
   /**
    * The discharge model dictates how the battery discharges over
    * time.
    */
   class CEPuck2BatteryDischargeModel : public CBaseConfigurableResource {
         
   public:

      const Real Px[5] = {1.0, -2.851222e-03, -1.819929e-05, 2.426572e-07, -6.673073e-10};  // P(x)
      const Real DELTA_S = 3.700821e-05;  // Static linear component
      //const Real DELTA_D = 1.721018e-03;  // Dynamic linear component
      const Real DELTA_D1 = 1.378337e-03;
      const Real DELTA_D2 = 8.847755e-07;
      const Real DELTA_D3 = 0.004842281;

      const Real TH1 = 0.8776304;
      const Real TH2 = 0.5936417;

      const Real MAX_X = 280; // Max distance before running out of battery (m)

      CEPuck2BatteryDischargeModel();

      virtual ~CEPuck2BatteryDischargeModel();

      virtual void Init(TConfigurationNode& t_tree) {}

      virtual void UpdatePos(const Real f_charge) {}

      virtual void Reset() {}

      virtual void Destroy() {}

      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);
         
      virtual void operator()() = 0;
         
   protected:
         
      CEPuck2BatteryEquippedEntity* m_pcBattery;
   };

   /****************************************/
   /****************************************/
   
   /**
    * For dynamic loading of battery discharge models.
    */
   typedef CFactory<CEPuck2BatteryDischargeModel> TFactoryBatteryDischargeModel;
   
#define REGISTER_BATTERY_DISCHARGE_MODEL(CLASSNAME, LABEL)  \
   REGISTER_SYMBOL(CEPuck2BatteryDischargeModel,                  \
                   CLASSNAME,                               \
                   LABEL,                                   \
                   "undefined",                             \
                   "undefined",                             \
                   "undefined",                             \
                   "undefined",                             \
                   "undefined")
   
   /**
    * A battery discharge model in which the charge decreases with both time and motion.
    *
    * In this model, the charge is calculated as follows:
    *
    * new charge = old charge - delta - pos_factor * (delta position)
    */
   class CEPuck2BatteryDischargeModelTimeMotion : public CEPuck2BatteryDischargeModel {
      
   public:
      
      CEPuck2BatteryDischargeModelTimeMotion() :
         m_psAnchor(NULL) {}

      virtual void Init(TConfigurationNode& t_tree);
      
      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);
      
      virtual void operator()();

      Real FindX(const Real f_charge);

      Real Delta(const Real f_charge, const Real f_delta);
      
      inline Real Eval(const Real P[5], const Real f_x) {
         return P[0] + P[1] * f_x + P[2] * f_x * f_x  +  P[3] * f_x * f_x * f_x +  P[4] * f_x * f_x * f_x * f_x;
      }

   protected:
      
      const SAnchor* m_psAnchor;
      CVector3 m_cOldPosition;
      CQuaternion m_cOldOrientation;
   };

   /****************************************/
   /****************************************/
   
}

#endif
