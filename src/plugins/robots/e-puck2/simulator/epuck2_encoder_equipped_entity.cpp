
/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_encoder_equipped_entity.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */
#include "epuck2_encoder_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CEPuck2EncoderEquippedEntity::SSensor::SSensor(const CWheeledEntity& c_wheels) :
      Wheels(c_wheels) {
      StepsPerCm =  1000.0 / (CRadians::TWO_PI.GetValue() * c_wheels.GetWheelRadius(0));
   }

   /****************************************/
   /****************************************/

   CEPuck2EncoderEquippedEntity::CEPuck2EncoderEquippedEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent) {
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2EncoderEquippedEntity::CEPuck2EncoderEquippedEntity(CComposableEntity* pc_parent,
                                                              const std::string& str_id) :
      CEntity(pc_parent, str_id) {
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2EncoderEquippedEntity::~CEPuck2EncoderEquippedEntity() {
      m_tSensors.clear();
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Parse basic entity stuff
          */
         CEntity::Init(t_tree);
         /*
          * Parse proximity sensors
          */
         /* Not adding any sensor is a fatal error */
         if(t_tree.NoChildren()) {
            THROW_ARGOSEXCEPTION("No sensors defined");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in encoder equipped entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderEquippedEntity::Enable() {
      CEntity::Enable();
//      for(size_t i = 0; i < m_tSensors.size(); ++i) {
//         m_tSensors[i]->Anchor.Enable();
//      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderEquippedEntity::Disable() {
      CEntity::Disable();
//      for(size_t i = 0; i < m_tSensors.size(); ++i) {
//         m_tSensors[i]->Anchor.Disable();
//      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2EncoderEquippedEntity::AddSensor(const CWheeledEntity& c_wheels) {
      m_tSensors.push_back(new SSensor(c_wheels));
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2EncoderEquippedEntity);

   /****************************************/
   /****************************************/

}
