/**
 * @file <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's works
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef CCI_EPUCK2_LEDS_ACTUATOR_H
#define CCI_EPUCK2_LEDS_ACTUATOR_H

namespace argos {
   class CCI_EPuck2LEDsActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/datatypes/color.h>

namespace argos {

   class CCI_EPuck2LEDsActuator : public CCI_Actuator {

   public:

      typedef std::vector<CColor> TSettings;

   public:

      CCI_EPuck2LEDsActuator() {}

      virtual ~CCI_EPuck2LEDsActuator() {}

      /**
       * @brief Returns the number of LEDs
       */
      size_t GetNumLEDs() const;

      /**
       * @brief Switches on and off the Red LED 1.
       *
       * @param b_state the state of the LED
       */
      virtual void SetRedLed1(const bool b_state);

      /**
       * @brief Switches on and off the Red LED 1.
       *
       * @param b_state the state of the LED
       */
      virtual void SetRedLed3(const bool b_state);

      /**
       * @brief Switches on and off the Red LED 1.
       *
       * @param b_state the state of the LED
       */
      virtual void SetRedLed5(const bool b_state);

      /**
       * @brief Switches on and off the Red LED 1.
       *
       * @param b_state the state of the LED
       */
      virtual void SetRedLed7(const bool b_state);

      /**
       * @brief Switches on and off the whole Red LEDs
       * All the Red LEDs will be on or off.
       *
       * @param b_state the state of the whole LEDs
       */
      virtual void SetAllRedLeds(const bool b_state);

      /**
       * @brief Sets the color of the RGB LED 2.
       *
       * @param c_color color to set
       */
      virtual void SetRGBLed2Color(const CColor& c_color);

      /**
       * @brief Sets the color of the RGB LED 4.
       *
       * @param c_color color to set
       */
      virtual void SetRGBLed4Color(const CColor& c_color);

      /**
       * @brief Sets the color of the RGB LED 6.
       *
       * @param c_color color to set
       */
      virtual void SetRGBLed6Color(const CColor& c_color);

      /**
       * @brief Sets the color of the RGB LED 8.
       *
       * @param c_color color to set
       */
      virtual void SetRGBLed8Color(const CColor& c_color);

      /**
       * @brief Sets the color of the whole RGB LEDs
       * All the RGB LEDs will be lit up in the same color.
       *
       * @param c_color color to set
       */
      virtual void SetAllRGBColors(const CColor& c_color);

      /**
       * @brief Switches on and off the Front LED.
       *
       * @param b_state the state of the LED
       */
      virtual void SetFrontLed(const bool b_state);

      /**
        * @brief Switches on and off the Body LED.
        *
        * @param b_state the state of the LED
        */
      virtual void SetBodyLed(const bool b_state);

      /**
       * @brief Switches off the whole LEDs
       * All the LEDs will be off.
       */
      virtual void SetAllBlack();


#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
#endif

   protected:

      TSettings m_tSettings;

   };

}

#endif
