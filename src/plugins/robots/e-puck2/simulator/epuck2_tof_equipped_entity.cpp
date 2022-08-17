
/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_tof_equipped_entity.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */
#include "epuck2_tof_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CEPuck2TOFEquippedEntity::SSensor::SSensor(const CVector3& c_offset,
                                                    const CVector3& c_direction,
                                                    Real f_range,
                                                    SAnchor& s_anchor) :
      Offset(c_offset),
      Direction(c_direction),
      Anchor(s_anchor) {
      Direction.Normalize();
      Direction *= f_range;
   }

   /****************************************/
   /****************************************/

   CEPuck2TOFEquippedEntity::CEPuck2TOFEquippedEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent) {
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2TOFEquippedEntity::CEPuck2TOFEquippedEntity(CComposableEntity* pc_parent,
                                                                  const std::string& str_id) :
      CEntity(pc_parent, str_id) {
      Disable();
   }

   /****************************************/
   /****************************************/

   CEPuck2TOFEquippedEntity::~CEPuck2TOFEquippedEntity() {
      while(! m_tSensors.empty()) {
         delete m_tSensors.back();
         m_tSensors.pop_back();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFEquippedEntity::Init(TConfigurationNode& t_tree) {
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
         /* Go through children */
         TConfigurationNodeIterator it;
         for(it = it.begin(&t_tree); it != it.end(); ++it) {
            std::string strAnchorId;
            GetNodeAttribute(*it, "anchor", strAnchorId);
            /*
             * NOTE: here we get a reference to the embodied entity
             * This line works under the assumption that:
             * 1. the CEPuck2TOFEquippedEntity has a parent;
             * 2. the parent has a child whose id is "body"
             * 3. the "body" is an embodied entity
             * If any of the above is false, this line will bomb out.
             */
            CEmbodiedEntity& cBody = GetParent().GetComponent<CEmbodiedEntity>("body");
            if(it->Value() == "sensor") {
               CVector3 cOff, cDir;
               Real fRange;
               GetNodeAttribute(*it, "offset", cOff);
               GetNodeAttribute(*it, "direction", cDir);
               GetNodeAttribute(*it, "range", fRange);
               AddSensor(cOff, cDir, fRange, cBody.GetAnchor(strAnchorId));
            }
            else {
               THROW_ARGOSEXCEPTION("Unrecognized tag \"" << it->Value() << "\"");
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in TOF equipped entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFEquippedEntity::Enable() {
      CEntity::Enable();
      for(size_t i = 0; i < m_tSensors.size(); ++i) {
         m_tSensors[i]->Anchor.Enable();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFEquippedEntity::Disable() {
      CEntity::Disable();
      for(size_t i = 0; i < m_tSensors.size(); ++i) {
         m_tSensors[i]->Anchor.Disable();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2TOFEquippedEntity::AddSensor(const CVector3& c_offset,
                                                  const CVector3& c_direction,
                                                  Real f_range,
                                                  SAnchor& s_anchor) {
      m_tSensors.push_back(new SSensor(c_offset, c_direction, f_range, s_anchor));
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CEPuck2TOFEquippedEntity);

   /****************************************/
   /****************************************/

}
