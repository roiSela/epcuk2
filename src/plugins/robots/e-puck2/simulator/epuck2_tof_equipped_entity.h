/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_tof_equipped_entity.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_TOF_EQUIPPED_ENTITY_H
#define EPUCK2_TOF_EQUIPPED_ENTITY_H

namespace argos {
   class CEPuck2TOFEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <map>

namespace argos {

   class CEPuck2TOFEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      struct SSensor {
         typedef std::vector<SSensor*> TList;

         CVector3 Offset;
         CVector3 Direction;
         SAnchor& Anchor;

         SSensor(const CVector3& c_offset,
                 const CVector3& c_direction,
                 Real f_range,
                 SAnchor& s_anchor);
      };

   public:

      CEPuck2TOFEquippedEntity(CComposableEntity* pc_parent);

      CEPuck2TOFEquippedEntity(CComposableEntity* pc_parent,
                                     const std::string& str_id);

      virtual ~CEPuck2TOFEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "tof_sensor";
      }

      virtual void Enable();

      virtual void Disable();

      inline size_t GetNumSensors() const {
         return m_tSensors.size();
      }

      inline SSensor& GetSensor(size_t un_idx) {
         return *m_tSensors[un_idx];
      }

      inline SSensor::TList& GetSensors() {
         return m_tSensors;
      }

      void AddSensor(const CVector3& c_offset,
                     const CVector3& c_direction,
                     Real f_range,
                     SAnchor& s_anchor);

   protected:

      /** The list of sensors */
      SSensor::TList m_tSensors;

   };

}

#endif
