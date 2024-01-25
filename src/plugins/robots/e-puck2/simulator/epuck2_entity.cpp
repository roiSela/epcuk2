/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.cpp>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_entity.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <argos3/plugins/simulator/entities/ground_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/entities/light_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>
#include <argos3/plugins/simulator/entities/proximity_sensor_equipped_entity.h>

#include "epuck2_led_equipped_entity.h"
#include "epuck2_tof_equipped_entity.h"
#include "epuck2_encoder_equipped_entity.h"
#include "epuck2_battery_equipped_entity.h"
#include "epuck2_camera_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   static const Real BODY_RADIUS                = 0.035f;

   static const Real INTERWHEEL_DISTANCE        = 0.053f;
   static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
   static const Real WHEEL_RADIUS               = 0.0205f;

   static const Real PROXIMITY_SENSOR_RING_ELEVATION       = 0.043f;
   static const Real PROXIMITY_SENSOR_RING_RADIUS          = BODY_RADIUS+0.001f;
   static const Real PROXIMITY_SENSOR_RING_RANGE           = 0.05f;
   static const Real LIGHT_SENSOR_RING_RANGE               = 0.50f;

   static const CRadians LED_RING_START_ANGLE   = CRadians::ZERO;
   static const Real LED_RING_RADIUS            = BODY_RADIUS + 0.002;
   static const Real LED_RING_ELEVATION         = 0.048f;
   static const Real RAB_ELEVATION              = LED_RING_ELEVATION;

   static const Real TOF_SENSOR_RANGE     = 2.0f;
   static const Real TOF_SENSOR_ELEVATION = 0.036f;
   static const Real TOF_SENSOR_OFFSET    = BODY_RADIUS - 0.003f;

   static const Real GREEN_LED_ELEVATION = PROXIMITY_SENSOR_RING_ELEVATION;

   static const Real RED_LED_ELEVATION   = 0.032f;
   static const Real RED_LED_SIDE        = -0.01f;
   static const Real RED_LED_POS         = 0.0345f;
   /****************************************/
   /****************************************/

   CEPuck2Entity::CEPuck2Entity() :
      CComposableEntity(NULL),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcGroundSensorEquippedEntity(NULL),
      m_pcEPuck2LEDEquippedEntity(NULL),
      m_pcLightSensorEquippedEntity(NULL),
      m_pcProximitySensorEquippedEntity(NULL),
      m_pcEPuck2TOFEquippedEntity(NULL),
      m_pcRABEquippedEntity(NULL),
      m_pcPerspectiveCameraEquippedEntity(NULL),
      m_pcWheeledEntity(NULL),
      m_pcBatteryEquippedEntity(NULL),
      m_pcEPuck2EncoderEquippedEntity(NULL) {
   }

   /****************************************/
   /****************************************/

   CEPuck2Entity::CEPuck2Entity(const std::string& str_id,
                              const std::string& str_controller_id,
                              const CVector3& c_position,
                              const CQuaternion& c_orientation,
                              Real f_rab_range,
                              size_t un_rab_data_size,
                              const std::string& str_bat_model,
                              const CRadians& c_perspcam_aperture,
                              Real f_perspcam_focal_length,
                              Real f_perspcam_range) :
      CComposableEntity(NULL, str_id),
      m_pcControllableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcGroundSensorEquippedEntity(NULL),
      m_pcEPuck2LEDEquippedEntity(NULL),
      m_pcLightSensorEquippedEntity(NULL),
      m_pcProximitySensorEquippedEntity(NULL),
      m_pcEPuck2TOFEquippedEntity(NULL),
      m_pcRABEquippedEntity(NULL),
      m_pcPerspectiveCameraEquippedEntity(NULL),
      m_pcWheeledEntity(NULL),
      m_pcBatteryEquippedEntity(NULL),
      m_pcEPuck2EncoderEquippedEntity(NULL) {
      try {
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this, "body_0", c_position, c_orientation);
         AddComponent(*m_pcEmbodiedEntity);
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         /* LED equipped entity */
         m_pcEPuck2LEDEquippedEntity = new CEPuck2LEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcEPuck2LEDEquippedEntity);
         m_pcEPuck2LEDEquippedEntity->AddLEDs(CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
                                              LED_RING_RADIUS,
                                              LED_RING_START_ANGLE,
                                              8,
                                              CVector3(0.0f, 0.0f, GREEN_LED_ELEVATION),
                                              CVector3(RED_LED_POS, RED_LED_SIDE, RED_LED_ELEVATION),
                                              m_pcEmbodiedEntity->GetOriginAnchor());
         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity = new CProximitySensorEquippedEntity(this, "proximity_0");
         AddComponent(*m_pcProximitySensorEquippedEntity);

         m_pcLightSensorEquippedEntity = new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);

         CRadians sensor_angle[8];
         sensor_angle[0] = -CRadians::PI / 10.5884f;
         sensor_angle[1] = -CRadians::PI / 3.5999f;
         sensor_angle[2] = -CRadians::PI_OVER_TWO; //side sensor
         sensor_angle[3] = -CRadians::PI / 1.2f; // back sensor
         sensor_angle[4] = CRadians::PI / 1.2f;  // back sensor
         sensor_angle[5] = CRadians::PI_OVER_TWO; //side sensor
         sensor_angle[6] = CRadians::PI / 3.5999f;
         sensor_angle[7] = CRadians::PI / 10.5884f;

         CRadians cAngle;
         CVector3 cOff, cDir, c_center = CVector3(0.0f, 0.0f, PROXIMITY_SENSOR_RING_ELEVATION);
         for(UInt32 i = 0; i < 8; ++i) {
            cAngle = sensor_angle[i];
            cAngle.SignedNormalize();
            cOff.Set(PROXIMITY_SENSOR_RING_RADIUS, 0.0f, 0.0f);
            cOff.RotateZ(cAngle);
            cOff += c_center;
            cDir.Set(PROXIMITY_SENSOR_RING_RANGE, 0.0f, 0.0f);
            cDir.RotateZ(cAngle);
            m_pcProximitySensorEquippedEntity->AddSensor(cOff, cDir, PROXIMITY_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
            m_pcLightSensorEquippedEntity->AddSensor(cOff, cDir, LIGHT_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         }

         /* TOF equipped entity */
         m_pcEPuck2TOFEquippedEntity = new CEPuck2TOFEquippedEntity(this, "tof_0");
         AddComponent(*m_pcEPuck2TOFEquippedEntity);
         cOff = CVector3(0.0f, 0.0f, TOF_SENSOR_ELEVATION);
         cDir = CVector3(0.0f, 0.0f, TOF_SENSOR_ELEVATION);
         c_center = CVector3(0.0f, 0.0f, TOF_SENSOR_ELEVATION);
         cOff.Set(TOF_SENSOR_OFFSET, 0.0f, 0.0f);
         cOff += c_center;
         cDir.Set(TOF_SENSOR_RANGE, 0.0f, 0.0f);
         m_pcEPuck2TOFEquippedEntity->AddSensor(cOff, cDir, TOF_SENSOR_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());

         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity = new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.03f, -0.009f),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.03f,  0.0f),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.03f,  0.009f),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         /* Encoder sensor equipped entity */
         m_pcEPuck2EncoderEquippedEntity = new CEPuck2EncoderEquippedEntity(this, "encoder_0");
         AddComponent(*m_pcEPuck2EncoderEquippedEntity);
         m_pcEPuck2EncoderEquippedEntity->AddSensor(*m_pcWheeledEntity);
         /* RAB equipped entity */
         m_pcRABEquippedEntity = new CRABEquippedEntity(this,
                                                        "rab_0",
                                                        un_rab_data_size,
                                                        f_rab_range,
                                                        m_pcEmbodiedEntity->GetOriginAnchor(),
                                                        *m_pcEmbodiedEntity,
                                                        CVector3(0.0f, 0.0f, RAB_ELEVATION));
         AddComponent(*m_pcRABEquippedEntity);
         /* Perspective camera equipped entity */
         m_pcEmbodiedEntity->EnableAnchor("perspective_camera");
         m_pcPerspectiveCameraEquippedEntity = new CEPuck2CameraEquippedEntity(this,
                                                                                    "perspective_camera_0",
                                                                                    c_perspcam_aperture,
                                                                                    f_perspcam_focal_length,
                                                                                    f_perspcam_range,
                                                                                    160, 120,
                                                                                    m_pcEmbodiedEntity->GetOriginAnchor());
         AddComponent(*m_pcPerspectiveCameraEquippedEntity);
         /* Battery equipped entity */
         m_pcBatteryEquippedEntity = new CEPuck2BatteryEquippedEntity(this, "battery_0", str_bat_model);
         AddComponent(*m_pcBatteryEquippedEntity);
         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this, "controller_0");
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->SetController(str_controller_id);
         /* Update components */
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2Entity::Init(TConfigurationNode& t_tree) {
      try {
         /*
          * Init parent
          */
         CComposableEntity::Init(t_tree);
         /*
          * Create and init components
          */
         /* Embodied entity */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         /* Wheeled entity and wheel positions (left, right) */
         m_pcWheeledEntity = new CWheeledEntity(this, "wheels_0", 2);
         AddComponent(*m_pcWheeledEntity);
         m_pcWheeledEntity->SetWheel(0, CVector3(0.0f,  HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         m_pcWheeledEntity->SetWheel(1, CVector3(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f), WHEEL_RADIUS);
         /* LED equipped entity */
         m_pcEPuck2LEDEquippedEntity = new CEPuck2LEDEquippedEntity(this, "leds_0");
         AddComponent(*m_pcEPuck2LEDEquippedEntity);
         m_pcEPuck2LEDEquippedEntity->AddLEDs(CVector3(0.0f, 0.0f, LED_RING_ELEVATION),
                                              LED_RING_RADIUS,
                                              LED_RING_START_ANGLE,
                                              8,
                                              CVector3(0.0f, 0.0f, GREEN_LED_ELEVATION),
                                              CVector3(RED_LED_POS, RED_LED_SIDE, RED_LED_ELEVATION),
                                              m_pcEmbodiedEntity->GetOriginAnchor());
         /* Proximity sensor equipped entity */
         m_pcProximitySensorEquippedEntity = new CProximitySensorEquippedEntity(this, "proximity_0");
         AddComponent(*m_pcProximitySensorEquippedEntity);
         /* Light sensor equipped entity */
         m_pcLightSensorEquippedEntity = new CLightSensorEquippedEntity(this, "light_0");
         AddComponent(*m_pcLightSensorEquippedEntity);

         CRadians sensor_angle[8];
         sensor_angle[0] = -CRadians::PI / 10.5884f;
         sensor_angle[1] = -CRadians::PI / 3.5999f;
         sensor_angle[2] = -CRadians::PI_OVER_TWO; //side sensor
         sensor_angle[3] = -CRadians::PI / 1.2f; // back sensor
         sensor_angle[4] = CRadians::PI / 1.2f;  // back sensor
         sensor_angle[5] = CRadians::PI_OVER_TWO; //side sensor
         sensor_angle[6] = CRadians::PI / 3.5999f;
         sensor_angle[7] = CRadians::PI / 10.5884f;

         CRadians cAngle;
         CVector3 cOff, cDir, c_center = CVector3(0.0f, 0.0f, PROXIMITY_SENSOR_RING_ELEVATION);
         for(UInt32 i = 0; i < 8; ++i) {
            cAngle = sensor_angle[i];
            cAngle.SignedNormalize();
            cOff.Set(PROXIMITY_SENSOR_RING_RADIUS, 0.0f, 0.0f);
            cOff.RotateZ(cAngle);
            cOff += c_center;
            cDir.Set(PROXIMITY_SENSOR_RING_RANGE, 0.0f, 0.0f);
            cDir.RotateZ(cAngle);
            m_pcProximitySensorEquippedEntity->AddSensor(cOff, cDir, PROXIMITY_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
            m_pcLightSensorEquippedEntity->AddSensor(cOff, cDir, LIGHT_SENSOR_RING_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         }
         /* TOF equipped entity */
         m_pcEPuck2TOFEquippedEntity = new CEPuck2TOFEquippedEntity(this, "tof_0");
         AddComponent(*m_pcEPuck2TOFEquippedEntity);
         cOff = CVector3(0.0f, 0.0f, TOF_SENSOR_ELEVATION);
         cDir = CVector3(0.0f, 0.0f, TOF_SENSOR_ELEVATION);
         c_center = CVector3(0.0f, 0.0f, TOF_SENSOR_ELEVATION);
         cOff.Set(TOF_SENSOR_OFFSET, 0.0f, 0.0f);
         cOff += c_center;
         cDir.Set(TOF_SENSOR_RANGE, 0.0f, 0.0f);
         m_pcEPuck2TOFEquippedEntity->AddSensor(cOff, cDir, TOF_SENSOR_RANGE, m_pcEmbodiedEntity->GetOriginAnchor());
         /* Ground sensor equipped entity */
         m_pcGroundSensorEquippedEntity = new CGroundSensorEquippedEntity(this, "ground_0");
         AddComponent(*m_pcGroundSensorEquippedEntity);
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.03f, -0.009f),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.03f,  0.0f),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         m_pcGroundSensorEquippedEntity->AddSensor(CVector2(0.03f,  0.009f),
                                                   CGroundSensorEquippedEntity::TYPE_GRAYSCALE,
                                                   m_pcEmbodiedEntity->GetOriginAnchor());
         /* Encoder sensor equipped entity */
         m_pcEPuck2EncoderEquippedEntity = new CEPuck2EncoderEquippedEntity(this, "encoder_0");
         AddComponent(*m_pcEPuck2EncoderEquippedEntity);
         m_pcEPuck2EncoderEquippedEntity->AddSensor(*m_pcWheeledEntity);
         /* RAB equipped entity */
         Real fRange = 0.8f;
         GetNodeAttributeOrDefault(t_tree, "rab_range", fRange, fRange);
         UInt32 unDataSize = 2;
         GetNodeAttributeOrDefault(t_tree, "rab_data_size", unDataSize, unDataSize);
         m_pcRABEquippedEntity = new CRABEquippedEntity(this,
                                                        "rab_0",
                                                        unDataSize,
                                                        fRange,
                                                        m_pcEmbodiedEntity->GetOriginAnchor(),
                                                        *m_pcEmbodiedEntity,
                                                        CVector3(0.0f, 0.0f, RAB_ELEVATION));
         AddComponent(*m_pcRABEquippedEntity);
         /* Perspective camera equipped entity */
         Real fPerspCamFocalLength = 0.035;
         Real fPerspCamRange = 1.0;
         CDegrees cAperture(18.5f);
         m_pcPerspectiveCameraEquippedEntity = new CEPuck2CameraEquippedEntity(this,
                                                                                    "perspective_camera_0",
                                                                                    ToRadians(cAperture),
                                                                                    fPerspCamFocalLength,
                                                                                    fPerspCamRange,
                                                                                    160, 120,
                                                                                    m_pcEmbodiedEntity->GetOriginAnchor());
         AddComponent(*m_pcPerspectiveCameraEquippedEntity);
         /* Battery equipped entity */
         m_pcBatteryEquippedEntity = new CEPuck2BatteryEquippedEntity(this, "battery_0");
         if(NodeExists(t_tree, "epuck2_battery"))
            m_pcBatteryEquippedEntity->Init(GetNode(t_tree, "epuck2_battery"));
         AddComponent(*m_pcBatteryEquippedEntity);
         /* Controllable entity
            It must be the last one, for actuators/sensors to link to composing entities correctly */
         m_pcControllableEntity = new CControllableEntity(this);
         AddComponent(*m_pcControllableEntity);
         m_pcControllableEntity->Init(GetNode(t_tree, "controller"));
         /* Update components */
         UpdateComponents();

      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEPuck2Entity::Reset() {
      /* Reset all components */
      CComposableEntity::Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CEPuck2Entity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

#define UPDATE(COMPONENT) if(COMPONENT->IsEnabled()) COMPONENT->Update();

   void CEPuck2Entity::UpdateComponents() {
      UPDATE(m_pcRABEquippedEntity);
      UPDATE(m_pcEPuck2LEDEquippedEntity);
      UPDATE(m_pcEPuck2TOFEquippedEntity);
      UPDATE(m_pcEPuck2EncoderEquippedEntity);
      UPDATE(m_pcBatteryEquippedEntity);
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CEPuck2Entity,
                   "e-puck2",
                   "Daniel H. Stolfi based on Carlo Pinciroli's work",
                   "1.0",
                   "The e-puck2 robot.",
                   "The e-puck2 is a open-hardware, extensible robot intended for education. In its\n"
                   "simplest form, it is a two-wheeled robot equipped with proximity sensors,\n"
                   "ground sensors, light sensors, a microphone, a front camera, and a ring of\n"
                   "red and RGB LEDs. More information is available at http://www.epuck.org\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <e-puck2 id=\"eb0\">\n"
                   "      <body position=\"0.4,2.3,0.0\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </e-puck2>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'body/position' attribute specifies the position of the pucktom point of the\n"
                   "e-puck2 in the arena. When the robot is untranslated and unrotated, the\n"
                   "pucktom point is in the origin and it is defined as the middle point between\n"
                   "the two wheels on the XY plane and the lowest point of the robot on the Z\n"
                   "axis, that is the point where the wheels touch the floor. The attribute values\n"
                   "are in the X,Y,Z order.\n"
                   "The 'body/orientation' attribute specifies the orientation of the e-puck. All\n"
                   "rotations are performed with respect to the pucktom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller/config' attribute is used to assign a controller to the\n"
                   "e-puck. The value of the attribute must be set to the id of a previously\n"
                   "defined controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "You can set the emission range of the range-and-bearing system. By default, a\n"
                   "message sent by an e-puck2 can be received up to 80cm. By using the 'rab_range'\n"
                   "attribute, you can change it to, i.e., 4m as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <e-puck2 id=\"eb0\" rab_range=\"4\">\n"
                   "      <body position=\"0.4,2.3,0.0\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </e-puck2>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also set the data sent at each time step through the range-and-bearing\n"
                   "system. By default, a message sent by an e-puck is 2 bytes long. By using the\n"
                   "'rab_data_size' attribute, you can change it to, i.e., 20 bytes as follows:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <e-puck2 id=\"eb0\" rab_data_size=\"20\">\n"
                   "      <body position=\"0.4,2.3,0.0\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "    </e-puck2>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can also configure the battery of the robot. By default, the battery never\n"
                   "depletes.\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <e-puck2 id=\"eb0\"\n"
                   "      <body position=\"0.4,2.3,0.0\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery start_charge=\"0.5\"/>\n"
                   "    </e-puck2>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "You can choose among several battery discharge models, they are enumerated\n"
                   "from the most simple to the more accurate. Note that the complexity (and elapsed time)"
                   "also increases.\n"
                   "- simple: (default) the battery depletes by two fixed rates, depending on if the robot\n"
                   "          is moving or static.\n"
                   "- linear: the battery depletes according to how the robot moves depending on\n"
                   "          its speed.\n"
                   "- approx: accurate model following the battery discharge curve.\n"
                   "- cubic: the most accurate model.\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <e-puck2 id=\"eb0\"\n"
                   "      <body position=\"0.4,2.3,0.0\" orientation=\"45,0,0\" />\n"
                   "      <controller config=\"mycntrl\" />\n"
                   "      <battery model=\"linear\"/>\n"
                   "    </e-puck2>\n"
                   "    ...\n"
                   "  </arena>\n\n",
                   "Usable"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CEPuck2Entity);

   /****************************************/
   /****************************************/

}
