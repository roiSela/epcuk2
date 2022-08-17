/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_encoder_equipped_entity.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_ENCODER_EQUIPPED_ENTITY_H
#define EPUCK2_ENCODER_EQUIPPED_ENTITY_H

namespace argos {
   class CEPuck2EncoderEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <map>

namespace argos {

   class CEPuck2EncoderEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      struct SSensor {
         typedef std::vector<SSensor*> TList;

         CWheeledEntity Wheels;
         Real StepsPerCm;  // constant depending on the wheel radius and steps per turn (1000)

         SSensor(const CWheeledEntity& c_wheels);
      };

   public:

      CEPuck2EncoderEquippedEntity(CComposableEntity* pc_parent);

      CEPuck2EncoderEquippedEntity(CComposableEntity* pc_parent,
                                   const std::string& str_id);

      virtual ~CEPuck2EncoderEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "encoder_sensor";
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

      void AddSensor(const CWheeledEntity& c_wheel);

   protected:

      /** The list of sensors */
      SSensor::TList m_tSensors;

   };

}

#endif
