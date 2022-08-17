/**
 * @file <argos3/plugins/robots/e-puck2/simulator/led_equipped_entity.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/media/led_medium.h>
#include "epuck2_led_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

    CEPuck2LEDEquippedEntity::SActuator::SActuator(CLEDEntity& c_led,
                                            const CVector3& c_offset,
                                            SAnchor& s_anchor,
                                            const ELEDType c_type) :
      LED(c_led),
      Offset(c_offset),
      Anchor(s_anchor),
      Type(c_type) {}

   /****************************************/
   /****************************************/

    CEPuck2LEDEquippedEntity::CEPuck2LEDEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
      Disable();
   }

   /****************************************/
   /****************************************/

    CEPuck2LEDEquippedEntity::CEPuck2LEDEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {
      Disable();
   }

   /****************************************/
   /****************************************/

    CEPuck2LEDEquippedEntity::~CEPuck2LEDEquippedEntity() {
      while(! m_tLEDs.empty()) {
         delete m_tLEDs.back();
         m_tLEDs.pop_back();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the led entries */
         CVector3 cPosition;
         CColor cColor;
         ELEDType cTypeId;
         TConfigurationNodeIterator itLED("led");
         for(itLED = itLED.begin(&t_tree);
             itLED != itLED.end();
             ++itLED) {
            /* Initialise the LED using the XML */
            CLEDEntity* pcLED = new CLEDEntity(this);
            pcLED->Init(*itLED);
            /* Parse the offset */
            CVector3 cOffset;
            GetNodeAttribute(*itLED, "offset", cOffset);
            /* Parse and look up the anchor */
            std::string strAnchorId;
            GetNodeAttribute(*itLED, "anchor", strAnchorId);
            std::string strTypeId;
            GetNodeAttribute(*itLED, "type", strTypeId);
            if (strTypeId=="body") {
                cTypeId = TYPE_BODY;
            } else if (strTypeId=="front") {
                cTypeId = TYPE_FRONT;
            } else {
                cTypeId = TYPE_RED;
            }

            /*
             * NOTE: here we get a reference to the embodied entity
             * This line works under the assumption that:
             * 1. the LEDEquippedEntity has a parent;
             * 2. the parent has a child whose id is "body"
             * 3. the "body" is an embodied entity
             * If any of the above is false, this line will bomb out.
             */
            CEmbodiedEntity& cBody = GetParent().GetComponent<CEmbodiedEntity>("body");
            /* Add the LED to this container */
            m_tLEDs.push_back(new SActuator(*pcLED, cOffset, cBody.GetAnchor(strAnchorId), cTypeId));
            AddComponent(*pcLED);
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize EPuck2 LED equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::Reset() {
      for(SActuator::TList::iterator it = m_tLEDs.begin();
          it != m_tLEDs.end();
          ++it) {
         (*it)->LED.Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::Enable() {
      /* Perform generic enable behavior */
      CComposableEntity::Enable();
      /* Enable anchors */
      for(size_t i = 0; i < m_tLEDs.size(); ++i) {
         m_tLEDs[i]->Anchor.Enable();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::Disable() {
      /* Perform generic disable behavior */
      CComposableEntity::Disable();
      /* Disable anchors */
      for(size_t i = 0; i < m_tLEDs.size(); ++i) {
         m_tLEDs[i]->Anchor.Disable();
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::AddLED(const CVector3& c_offset,
                                   SAnchor& s_anchor,
                                   const ELEDType c_type) {
      CLEDEntity* pcLED =
         new CLEDEntity(
            this,
            std::string("led_") + ToString(m_tLEDs.size()),
            c_offset,
            CColor::BLACK);
      m_tLEDs.push_back(new SActuator(*pcLED, c_offset, s_anchor, c_type));
      AddComponent(*pcLED);
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::AddLEDs(const CVector3& c_center,
                                       Real f_radius,
                                       const CRadians& c_start_angle,
                                       UInt32 un_num_leds,
                                       SAnchor& s_anchor,
                                       const CVector3& c_body_offset,
                                       SAnchor& s_body_anchor,
                                       const CVector3& c_front_offset,
                                       SAnchor& s_front_anchor) {
      CRadians cLEDSpacing = CRadians::TWO_PI / un_num_leds;
      CRadians cAngle;
      CVector3 cOffset;
      for(UInt32 i = 0; i < un_num_leds; ++i) {
         cAngle = c_start_angle + i * cLEDSpacing;
         cAngle.SignedNormalize();
         cOffset.Set(f_radius, 0.0f, 0.0f);
         cOffset.RotateZ(cAngle);
         cOffset += c_center;
         if (i % 2 == 0) {
             AddLED(cOffset, s_anchor, TYPE_RED);
         } else {
             AddLED(cOffset, s_anchor, TYPE_RGB);
         }
      }
      AddLED(c_body_offset, s_body_anchor, TYPE_BODY);
      AddLED(c_front_offset, s_front_anchor, TYPE_FRONT);
   }

   /****************************************/
   /****************************************/

   CLEDEntity& CEPuck2LEDEquippedEntity::GetLED(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tLEDs.size(),
                   "CEPuck2LEDEquippedEntity::GetLED(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLEDs.size() = " <<
                   m_tLEDs.size());
      return m_tLEDs[un_index]->LED;
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetLEDOffset(UInt32 un_index,
                                         const CVector3& c_offset) {
      ARGOS_ASSERT(un_index < m_tLEDs.size(),
                   "CEPuck2LEDEquippedEntity::SetLEDPosition(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tLEDs.size() = " <<
                   m_tLEDs.size());
      m_tLEDs[un_index]->Offset = c_offset;
   }


   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetRGBLEDColor(UInt32 un_index,
                       const CColor& c_color) {
       ARGOS_ASSERT(m_tLEDs.size() > 0 && (un_index == 1 || un_index == 3 || un_index == 5 || un_index == 7),
                    "CEPuck2LEDEquippedEntity::SetRGBLEDColor(), id=\"" <<
                    GetId() <<
                    "\": index out of bounds: un_index = " <<
                    un_index <<
                    ", m_tLEDs.size() = " <<
                    m_tLEDs.size() << " RGBLEDs are 1, 3, 5, and 7");
       m_tLEDs[un_index]->LED.SetColor(c_color);

   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetRedLED(UInt32 un_index,
                  const bool c_state) {
       ARGOS_ASSERT(m_tLEDs.size() > 0 && (un_index == 0 || un_index == 2 || un_index == 4 || un_index == 6),
                    "CEPuck2LEDEquippedEntity::SetRedLED(), id=\"" <<
                    GetId() <<
                    "\": index out of bounds: un_index = " <<
                    un_index <<
                    ", m_tLEDs.size() = " <<
                    m_tLEDs.size() << " RedLEDs are 0, 2, 4, and 6");
       m_tLEDs[un_index]->LED.SetColor(c_state ? CColor::RED : CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetFrontLED(const bool c_state) {
       ARGOS_ASSERT(m_tLEDs.size() > 0,
                    "CEPuck2LEDEquippedEntity::SetFrontLED(), id=\"" <<
                    GetId() <<
                    "\": there is no LEDs, m_tLEDs.size() = " <<
                    m_tLEDs.size());
       m_tLEDs[8]->LED.SetColor(c_state ? CColor::RED : CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetBodyLED(const bool c_state) {
       ARGOS_ASSERT(m_tLEDs.size() > 0,
                    "CEPuck2LEDEquippedEntity::SetBodyLED(), id=\"" <<
                    GetId() <<
                    "\": there is no LEDs, m_tLEDs.size() = " <<
                    m_tLEDs.size());
       m_tLEDs[9]->LED.SetColor(c_state ? CColor::GREEN : CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetAllLEDsColors(const std::vector<CColor>& vec_colors) {
       ARGOS_ASSERT(m_tLEDs.size() == 10,
                   "CEPuck2LEDEquippedEntity::SetAllLEDsColors(), id=\"" <<
                   GetId() << "\": there is not 10 LEDs, m_tLEDs.size() = " <<
                   m_tLEDs.size());
       ARGOS_ASSERT((vec_colors[0] == CColor::BLACK || vec_colors[0] == CColor::RED) &&
                    (vec_colors[2] == CColor::BLACK || vec_colors[2] == CColor::RED) &&
                    (vec_colors[4] == CColor::BLACK || vec_colors[4] == CColor::RED) &&
                    (vec_colors[6] == CColor::BLACK || vec_colors[6] == CColor::RED) &&
                    (vec_colors[8] == CColor::BLACK || vec_colors[8] == CColor::RED) &&
                    (vec_colors[9] == CColor::BLACK || vec_colors[9] == CColor::GREEN),
                   "CEPuck2LEDEquippedEntity::SetAllLEDsColors(), id=\"" <<
                   GetId() << "\": the provided colours are invalid.");
      if(vec_colors.size() == m_tLEDs.size()) {
         for(UInt32 i = 0; i < vec_colors.size(); ++i) {
            m_tLEDs[i]->LED.SetColor(vec_colors[i]);
         }
      }
      else {
         THROW_ARGOSEXCEPTION(
            "CEPuck2LEDEquippedEntity::SetAllLEDsColors(), id=\"" <<
            GetId() <<
            "\": number of LEDs (" <<
            m_tLEDs.size() <<
            ") is lower than the passed colour vector size (" <<
            vec_colors.size() <<
            ")");
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::UpdateComponents() {
      /* LED position wrt global reference frame */
      CVector3 cLEDPosition;
      for(UInt32 i = 0; i < m_tLEDs.size(); ++i) {
         if(m_tLEDs[i]->LED.IsEnabled()) {
            cLEDPosition = m_tLEDs[i]->Offset;
            cLEDPosition.Rotate(m_tLEDs[i]->Anchor.Orientation);
            cLEDPosition += m_tLEDs[i]->Anchor.Position;
            m_tLEDs[i]->LED.SetPosition(cLEDPosition);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2LEDEquippedEntity::SetMedium(CLEDMedium& c_medium) {
      for(UInt32 i = 0; i < m_tLEDs.size(); ++i) {
         m_tLEDs[i]->LED.SetMedium(c_medium);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CEPuck2LEDEquippedEntity);

   /****************************************/
   /****************************************/

}
