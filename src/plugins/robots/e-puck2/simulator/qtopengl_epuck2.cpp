/**
 * @file <argos3/plugins/robots/e-puck2/simulator/qtopengl_epuck2.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "qtopengl_epuck2.h"
#include "epuck2_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

#include "epuck2_led_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   /* All measures are in meters */

   static const Real WHEEL_DIAMETER              = 0.041f;
   static const Real WHEEL_RADIUS                = WHEEL_DIAMETER * 0.5f;
   static const Real WHEEL_WIDTH                 = 0.01f;
   static const Real HALF_WHEEL_WIDTH            = WHEEL_WIDTH * 0.5f;
   static const Real INTERWHEEL_DISTANCE         = 0.053f;
   static const Real HALF_INTERWHEEL_DISTANCE    = INTERWHEEL_DISTANCE * 0.5f;

   static const Real CHASSIS_ELEVATION           = 0.005f;                            // to be checked!
   static const Real HALF_CHASSIS_LENGTH         = 0.0275f;                             // to be checked!
   static const Real HALF_CHASSIS_WIDTH          = HALF_INTERWHEEL_DISTANCE - HALF_WHEEL_WIDTH;

   static const Real BODY_RADIUS                 = 0.035f;
   static const Real BODY_ELEVATION              = WHEEL_DIAMETER; // + CHASSIS_ELEVATION; // to be checked!
   static const Real BODY_HEIGHT                 = 0.005f;                              // to be checked!

   static const Real LED_ELEVATION               = BODY_ELEVATION + BODY_HEIGHT;
   static const Real LED_HEIGHT                  = 0.005;                               // to be checked!
   static const Real LED_UPPER_RING_INNER_RADIUS = 0.8 * BODY_RADIUS;

   static const Real FRONT_LED_HEIGHT    = 0.005f;
   static const Real FRONT_LED_SIDE      = 0.01f;
   static const Real FRONT_LED_HALF_SIDE = FRONT_LED_SIDE * 0.5f;
   static const Real FRONT_LED_ELEVATION = 0.032f;
   static const Real FRONT_LED_OFFSET    = HALF_CHASSIS_LENGTH;
   static const Real FRONT_LED_DEV       = -0.003f;

   /****************************************/
   /****************************************/

   CQTOpenGLEPuck2::CQTOpenGLEPuck2() :
      m_unVertices(40),
      m_fLEDAngleSlice(360.0f / 16.0f) {
      /* Reserve the needed display lists */
      m_unLists = glGenLists(6);

      /* Assign indices for better referencing (later) */
      m_unWheelList   = m_unLists;
      m_unChassisList = m_unLists + 1;
      m_unBodyList    = m_unLists + 2;
      m_unLEDList     = m_unLists + 3;
      m_unGapList     = m_unLists + 4;
      m_unFrontLED    = m_unLists + 5;

      /* Create the wheel display list */
      glNewList(m_unWheelList, GL_COMPILE);
      RenderWheel();
      glEndList();

      /* Create the body display list */
      glNewList(m_unBodyList, GL_COMPILE);
      RenderBody();
      glEndList();

      /* Create the chassis display list */
      glNewList(m_unChassisList, GL_COMPILE);
      RenderChassis();
      glEndList();

      /* Create the LED display list */
      glNewList(m_unLEDList, GL_COMPILE);
      RenderLED();
      glEndList();

      /* Create the Gap between LED display list */
      glNewList(m_unGapList, GL_COMPILE);
      RenderGap();
      glEndList();

      /* Create the front LED list */
      glNewList(m_unFrontLED, GL_COMPILE);
      RenderFrontLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLEPuck2::~CQTOpenGLEPuck2() {
      glDeleteLists(m_unLists, 4);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::Draw(CEPuck2Entity& c_entity) {
      CEPuck2LEDEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      const CColor& cBodyColor = cLEDEquippedEntity.GetLED(9).GetColor();
      if (cBodyColor == CColor::BLACK) {
         SetGreenPlasticMaterial();
      } else {
         SetLEDMaterial(0.0f, 1.0f, 0.0f);
      }
      /* Place the chassis */
      glCallList(m_unChassisList);
      /* Place the body */
      glCallList(m_unBodyList);
      /* Place the wheels */
      glPushMatrix();
      glTranslatef(0.0f, HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      glPushMatrix();
      glTranslatef(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      /* Place the LEDs */
      glPushMatrix();
      for(UInt32 i = 0; i < 8; i++) {
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         glRotatef(-m_fLEDAngleSlice, 0.0f, 0.0f, 1.0f);
         SetLEDMaterial(cColor.GetRed() / 255.0f ,
                        cColor.GetGreen() / 255.0f,
                        cColor.GetBlue() / 255.0f) ;
         glCallList(m_unLEDList);

         glRotatef(-m_fLEDAngleSlice, 0.0f, 0.0f, 1.0f);
         SetWhitePlasticMaterial();
         glCallList(m_unGapList);
      }
      glPopMatrix();
      /* Front LED */
      glPushMatrix();
      glRotatef(-15.0f, 0.0f, 0.0f, 1.0f);
      const CColor& cFrontColor = cLEDEquippedEntity.GetLED(8).GetColor();
      SetLEDMaterial(cFrontColor.GetRed() / 255.0f,
                     cFrontColor.GetGreen() / 255.0f,
                     cFrontColor.GetBlue() / 255.0f);
      glCallList(m_unFrontLED);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::SetWhitePlasticMaterial() {
      const GLfloat pfColor[]     = {   0.95f, 0.95f, 0.95f, 1.0f };
      const GLfloat pfSpecular[]  = {   1.0f,  1.0f,  1.0f,  1.0f };
      const GLfloat pfShininess[] = {  50.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::SetGreenPlasticMaterial() {
      const GLfloat pfColor[]     = {   0.0f, 0.8f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.5f, 0.5f, 0.5f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::SetRedPlasticMaterial() {
      const GLfloat pfColor[]     = {   1.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::SetCircuitBoardMaterial() {
      const GLfloat pfColor[]     = { 0.0f, 0.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = { 0.5f, 0.5f, 1.0f, 1.0f };
      const GLfloat pfShininess[] = { 10.0f                  };
      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::SetLEDMaterial(GLfloat f_red,
                                       GLfloat f_green,
                                       GLfloat f_blue) {
      const GLfloat pfColor[]     = { f_red, f_green, f_blue, 1.0f };
      const GLfloat pfSpecular[]  = {  0.0f,    0.0f,   0.0f, 1.0f };
      const GLfloat pfShininess[] = {  0.0f                        };
      const GLfloat pfEmission[]  = { f_red, f_green, f_blue, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::RenderWheel() {
      /* Set material */
      SetRedPlasticMaterial();
      /* Right side */
      CVector2 cVertex(WHEEL_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector3 cNormal(-1.0f, -1.0f, 0.0f);
      cNormal.Normalize();
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Left side */
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cNormal.Set(-1.0f, 1.0f, 0.0f);
      cNormal.Normalize();
      cAngle = -cAngle;
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Tire */
      cNormal.Set(1.0f, 0.0f, 0.0f);
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cAngle = -cAngle;
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         glVertex3f(cVertex.GetX(),  HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::RenderChassis() {
      /* This part covers the bottom face (parallel to XY) */
      glBegin(GL_QUADS);
      /* Bottom face */
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUAD_STRIP);
      /* Starting side */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* South face */
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* East face */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* North face */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* West face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::RenderBody() {
      /* Set material */
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(-CRadians::TWO_PI / m_unVertices);
      /* Bottom part */
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, -1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Side surface */
      cAngle = -cAngle;
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
      /* Top part */
      SetGreenPlasticMaterial();
      glBegin(GL_POLYGON);
      cVertex.Set(LED_UPPER_RING_INNER_RADIUS, 0.0f);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Triangle to set the direction */
      SetLEDMaterial(1.0f, 1.0f, 0.0f);
      glBegin(GL_TRIANGLES);
      glVertex3f( BODY_RADIUS * 0.7,               0.0f, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
      glVertex3f(-BODY_RADIUS * 0.7,  BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
      glVertex3f(-BODY_RADIUS * 0.7, -BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT + 0.001f);
      glEnd();
   }

   /****************************************/
   /****************************************/
    void CQTOpenGLEPuck2::RenderFrontLED() {
      glBegin(GL_TRIANGLES);
      /* Top */
      glVertex3f(FRONT_LED_HEIGHT + FRONT_LED_OFFSET,                 0.0f + FRONT_LED_DEV, FRONT_LED_ELEVATION                        );
      glVertex3f(                   FRONT_LED_OFFSET,  FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION + FRONT_LED_HALF_SIDE);
      glVertex3f(                   FRONT_LED_OFFSET, -FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION + FRONT_LED_HALF_SIDE);
      /* Bottom */
      glVertex3f(FRONT_LED_HEIGHT + FRONT_LED_OFFSET,                 0.0f + FRONT_LED_DEV, FRONT_LED_ELEVATION                        );
      glVertex3f(                   FRONT_LED_OFFSET, -FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION - FRONT_LED_HALF_SIDE);
      glVertex3f(                   FRONT_LED_OFFSET,  FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION - FRONT_LED_HALF_SIDE);
      /* Left */
      glVertex3f(FRONT_LED_HEIGHT + FRONT_LED_OFFSET,                 0.0f + FRONT_LED_DEV, FRONT_LED_ELEVATION                        );
      glVertex3f(                   FRONT_LED_OFFSET,  FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION - FRONT_LED_HALF_SIDE);
      glVertex3f(                   FRONT_LED_OFFSET,  FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION + FRONT_LED_HALF_SIDE);
      /* Right */
      glVertex3f(FRONT_LED_HEIGHT + FRONT_LED_OFFSET,                 0.0f + FRONT_LED_DEV, FRONT_LED_ELEVATION                        );
      glVertex3f(                   FRONT_LED_OFFSET, -FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION + FRONT_LED_HALF_SIDE);
      glVertex3f(                   FRONT_LED_OFFSET, -FRONT_LED_HALF_SIDE + FRONT_LED_DEV, FRONT_LED_ELEVATION - FRONT_LED_HALF_SIDE);
      glEnd();
   }


   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::RenderLED() {
      /* Side surface */
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector2 cNormal(1.0f, 0.0f);
      glBegin(GL_QUAD_STRIP);
      cVertex.Rotate(CRadians::PI / 10.0f);
      cNormal.Rotate(CRadians::PI / 10.0f);
      for(GLuint i = 0; i <= m_unVertices / 32; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
      /* Top surface  */
      cVertex.Set(BODY_RADIUS, 0.0f);
      CVector2 cVertex2(LED_UPPER_RING_INNER_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      cVertex.Rotate(CRadians::PI / 10.0f);
      cVertex2.Rotate(CRadians::PI / 10.0f);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices / 32; i++) {
         glVertex3f(cVertex2.GetX(), cVertex2.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         cVertex.Rotate(cAngle);
         cVertex2.Rotate(cAngle);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLEPuck2::RenderGap() {
      /* Side surface */
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector2 cNormal(1.0f, 0.0f);
      glBegin(GL_QUAD_STRIP);
      cVertex.Rotate(CRadians::PI / 3.65f);
      cNormal.Rotate(CRadians::PI / 3.65f);
      for(GLuint i = 0; i <= m_unVertices / 10; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), LED_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
      /* Top surface  */
      cVertex.Set(BODY_RADIUS, 0.0f);
      CVector2 cVertex2(LED_UPPER_RING_INNER_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      cVertex.Rotate(CRadians::PI / 3.65f);
      cVertex2.Rotate(CRadians::PI / 3.65f);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices / 10; i++) {
         glVertex3f(cVertex2.GetX(), cVertex2.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT + LED_HEIGHT);
         cVertex.Rotate(cAngle);
         cVertex2.Rotate(cAngle);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawEPuck2Normal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CEPuck2Entity& c_entity) {
         static CQTOpenGLEPuck2 m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawEPuck2Selected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CEPuck2Entity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawEPuck2Normal, CEPuck2Entity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawEPuck2Selected, CEPuck2Entity);

   /****************************************/
   /****************************************/

}
