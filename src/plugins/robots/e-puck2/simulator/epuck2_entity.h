/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK_ENTITY2_H
#define EPUCK_ENTITY2_H

namespace argos {
   class CControllableEntity;
   class CEmbodiedEntity;
   class CEPuckEntity;
   class CGroundSensorEquippedEntity;
   class CEPuck2LEDEquippedEntity;
   class CLightSensorEquippedEntity;
   class CProximitySensorEquippedEntity;
   class CEPuck2TOFEquippedEntity;
   class CRABEquippedEntity;
   class CEPuck2CameraEquippedEntity;
   class CEPuck2BatteryEquippedEntity;
   class CEPuck2EncoderEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>

namespace argos {

   class CEPuck2Entity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      CEPuck2Entity();

      CEPuck2Entity(const std::string& str_id,
                   const std::string& str_controller_id,
                   const CVector3& c_position = CVector3(),
                   const CQuaternion& c_orientation = CQuaternion(),
                   Real f_rab_range = 0.8f,
                   size_t un_rab_data_size = 2,
                   const std::string& str_bat_model = "",
                   const CRadians& c_perspcam_aperture = ToRadians(CDegrees(15.0f)),
                   Real f_perspcam_focal_length = 0.035f,
                   Real f_perspcam_range = 1.0f);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      inline CControllableEntity& GetControllableEntity() {
         return *m_pcControllableEntity;
      }

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CGroundSensorEquippedEntity& GetGroundSensorEquippedEntity() {
         return *m_pcGroundSensorEquippedEntity;
      }

      inline CEPuck2LEDEquippedEntity& GetLEDEquippedEntity() {
         return *m_pcEPuck2LEDEquippedEntity;
      }

      inline CLightSensorEquippedEntity& GetLightSensorEquippedEntity() {
         return *m_pcLightSensorEquippedEntity;
      }

      inline CProximitySensorEquippedEntity& GetProximitySensorEquippedEntity() {
         return *m_pcProximitySensorEquippedEntity;
      }

      inline CEPuck2TOFEquippedEntity& GetEPuck2TOFEquippedEntity() {
         return *m_pcEPuck2TOFEquippedEntity;
      }

      inline CRABEquippedEntity& GetRABEquippedEntity() {
         return *m_pcRABEquippedEntity;
      }

      inline CWheeledEntity& GetWheeledEntity() {
         return *m_pcWheeledEntity;
      }

      inline CEPuck2EncoderEquippedEntity& GetEPuck2EncoderEquippedEntity() {
          return *m_pcEPuck2EncoderEquippedEntity;
      }

      inline CEPuck2BatteryEquippedEntity& GetBatterySensorEquippedEntity() {
          return *m_pcBatteryEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "e-puck2";
      }

   private:

      void SetLEDPosition();

   private:

      CControllableEntity*                   m_pcControllableEntity;
      CEmbodiedEntity*                       m_pcEmbodiedEntity;
      CGroundSensorEquippedEntity*           m_pcGroundSensorEquippedEntity;
      CEPuck2LEDEquippedEntity*              m_pcEPuck2LEDEquippedEntity;
      CLightSensorEquippedEntity*            m_pcLightSensorEquippedEntity;
      CProximitySensorEquippedEntity*        m_pcProximitySensorEquippedEntity;
      CEPuck2TOFEquippedEntity*              m_pcEPuck2TOFEquippedEntity;
      CRABEquippedEntity*                    m_pcRABEquippedEntity;
      CEPuck2CameraEquippedEntity*           m_pcPerspectiveCameraEquippedEntity;
      CWheeledEntity*                        m_pcWheeledEntity;
      CEPuck2BatteryEquippedEntity*          m_pcBatteryEquippedEntity;
      CEPuck2EncoderEquippedEntity*          m_pcEPuck2EncoderEquippedEntity;
   };

}

#endif
