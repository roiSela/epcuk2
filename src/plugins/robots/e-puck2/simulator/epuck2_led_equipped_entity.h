/**
 * @file <argos3/plugins/robots/e-puck2/simulator/led_equipped_entity.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_LED_EQUIPPED_ENTITY_H
#define EPUCK2_LED_EQUIPPED_ENTITY_H

namespace argos {
   class CEPuck2LEDEquippedEntity;
   class CLEDEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <map>

namespace argos {

   /**
    * A container of CLEDEntity.
    * <p>
    * This is a convenience class that acts a container of CLEDEntity objects. It
    * is mostly useful when a robot is equipped with a number of LEDs, and you
    * want to manage them comfortably.
    * </p>
    * <p>
    * You can define a positional entity as the <em>reference</em> of this entity.
    * In this way, if the reference entity moves, this entity will follow automatically.
    * The contained LEDs will also move accordingly. If you don't define a reference
    * entity, the LEDs won't move.
    * </p>
    * @see CLEDEntity
    */
   class CEPuck2LEDEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CEPuck2LEDEquippedEntity*> TMap;

   public:
      enum ELEDType {
         TYPE_BODY = 0,
         TYPE_FRONT,
         TYPE_RED,
         TYPE_RGB
      };

      struct SActuator {
         typedef std::vector<SActuator*> TList;

         CLEDEntity& LED;
         CVector3 Offset;
         SAnchor& Anchor;
         ELEDType Type;


         SActuator(CLEDEntity& c_led,
                   const CVector3& c_offset,
                   SAnchor& s_anchor,
                   const ELEDType c_type);
      };

   public:

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       */
      CEPuck2LEDEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * @param pc_parent The parent entity.
       * @param str_id The id of this entity.
       */
      CEPuck2LEDEquippedEntity(CComposableEntity* pc_parent,
                         const std::string& str_id);

      /**
       * Class destructor.
       */
      ~CEPuck2LEDEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Enable();

      virtual void Disable();

      /**
       * Adds a ring of LEDs to this entity.
       * @param c_center The position of LED ring centre wrt the reference entity.
       * @param f_radius The radius of the LED ring.
       * @param c_start_angle The angle at which the first LED must be placed. Expressed wrt the local entity <em>x</em>-axis.
       * @param un_num_leds the number of LEDs
       * @param c_body_center The position of LED ring centre wrt the reference entity.
       * @param c_front_offset The offset for the front LED.
       * @param s_anchor The anchor of the LEDs.
       */
      void AddLEDs(const CVector3& c_center,
            Real f_radius,
            const CRadians& c_start_angle,
            UInt32 un_num_leds,
            const CVector3& c_body_center,
            const CVector3& c_front_offset,
            SAnchor& s_anchor);


      /**
       * Returns an LED by numeric index.
       * @param un_index The index of the wanted LED.
       * @return An LED by numeric index.
       * @see GetAllLEDs()
       */
      CLEDEntity& GetLED(UInt32 un_index);

      /**
       * Returns all the LEDs.
       * @return All the LEDs.
       * @see GetLED()
       */
      inline SActuator::TList& GetLEDs() {
         return m_tLEDs;
      }

      /**
       * Returns the offset position of the given LED.
       * The actual position of an LED is calculated as the vector sum of
       * the offset position and the anchor.
       * @return The offset position of the given LED.
       */
      inline const CVector3& GetLEDOffset(size_t un_idx) const {
         ARGOS_ASSERT(un_idx < m_tLEDs.size(),
                      "CLEDEquippedEntity::GetLEDOffset() : index " <<
                      un_idx <<
                      " out of bounds [0:" <<
                      m_tLEDs.size()-1 <<
                      "]" );
         return m_tLEDs[un_idx]->Offset;
      }

      /**
       * Sets the position of an LED.
       * @param un_index The index of the wanted LED.
       * @param c_offset The position of the LED wrt the anchor.
       */
      void SetLEDOffset(UInt32 un_index,
                        const CVector3& c_offset);

      /**
       * Sets the colour of an RGB LED.
       * @param un_index The index of the wanted RGB LED.
       * @param c_color The colour of the LED.
       */
      void SetRGBLEDColor(UInt32 un_index,
                          const CColor& c_color);

      /**
       * Sets the state of an LED.
       * @param un_index The index of the wanted LED.
       * @param c_state The state of the LED.
       */
      void SetRedLED(UInt32 un_index,
                     const bool c_state);

      /**
       * Sets the state of the Front LED.
       * @param c_state The state of the Front LED.
       */
      void SetFrontLED(const bool c_state);

      /**
       * Sets the state of the Body LED.
       * @param c_state The state of the Body LED.
       */
      void SetBodyLED(const bool c_state);

      /**
       * Sets the colour of all the LEDs to the given setting.
       * @param vec_colors A vector containing the colours of the LEDs.
       * @see SetAllLEDsColors()
       * @throws CARGoSException if the size of the passed vector is different from
       * the number of LED or if the colours are wrong.
       */
      void SetAllLEDsColors(const std::vector<CColor>& vec_colors);

      /**
       * Sets the medium associated to this entity.
       * @param c_medium The medium to associate to this entity.
       * @see CLEDMedium
       */
      void SetMedium(CLEDMedium& c_medium);

      virtual std::string GetTypeDescription() const {
         return "epuck2_leds";
      }

   protected:

      virtual void UpdateComponents();

   protected:

      /** List of the LEDs managed by this entity */
      SActuator::TList m_tLEDs;

   private:
       void AddLED(const CVector3& c_offset,
                   SAnchor& s_anchor,
                   const ELEDType c_type);

   };


}

#endif
