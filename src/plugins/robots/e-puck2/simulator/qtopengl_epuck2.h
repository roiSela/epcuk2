/**
 * @file <argos3/plugins/robots/e-puck2/simulator/qtopengl_epuck2.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef QTOPENGL_EPUCK2_H
#define QTOPENGL_EPUCK2_H

namespace argos {
   class CQTOpenGLEPuck2;
   class CEPuck2Entity;
}

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLEPuck2 {

   public:

      CQTOpenGLEPuck2();

      virtual ~CQTOpenGLEPuck2();

      virtual void Draw(CEPuck2Entity& c_entity);

   protected:

      /** Sets a white plastic material */
      void SetWhitePlasticMaterial();
      /** Sets a green plastic material */
      void SetGreenPlasticMaterial();
      /** Sets a red plastic material */
      void SetRedPlasticMaterial();
      /** Sets a circuit board material */
      void SetCircuitBoardMaterial();
      /** Sets a colored LED material */
      void SetLEDMaterial(GLfloat f_red,
                          GLfloat f_green,
                          GLfloat f_blue);

      /** Renders a wheel */
      void RenderWheel();
      /** Renders the chassis */
      void RenderChassis();
      /** Renders the body */
      void RenderBody();
      /** A single LED of the ring */
      void RenderLED();
      /** A single GAP between LEDs of the ring */
      void RenderGap();
      /** The front LED */
      void RenderFrontLED();

   private:

      /** Start of the display list index */
      GLuint m_unLists;

      /** E-puck wheel */
      GLuint m_unWheelList;

      /** Chassis display list */
      GLuint m_unChassisList;

      /** Body display list */
      GLuint m_unBodyList;

      /** LED display list */
      GLuint m_unLEDList;

      /** LED display list */
      GLuint m_unGapList;

      /** Front LED list */
      GLuint m_unFrontLED;

      /** Number of vertices to display the round parts
          (wheels, chassis, etc.) */
      GLuint m_unVertices;

      /* Angle gap between two leds */
      GLfloat m_fLEDAngleSlice;


   };

}

#endif
