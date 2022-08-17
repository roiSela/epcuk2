/**
 * @file <argos3/plugins/robots/e-puck2/simulator/epuck2_colored_blob_perspective_camera_default_sensor.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_COLORED_BLOB_PERSPECTIVE_CAMERA_DEFAULT_SENSOR_H
#define EPUCK2_COLORED_BLOB_PERSPECTIVE_CAMERA_DEFAULT_SENSOR_H

namespace argos {
   class CColoredBlobPerspectiveCameraDefaultSensor;
   class CEPuck2CameraEquippedEntity;
   class CLEDEntity;
   class CControllableEntity;
   class CEPuck2PerspectiveCameraLEDCheckOperation;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_perspective_camera_sensor.h>
#include <argos3/core/control_interface/ci_sensor.h>

namespace argos {

   class CEPuck2ColoredBlobPerspectiveCameraDefaultSensor : public CCI_ColoredBlobPerspectiveCameraSensor,
                                                            public CSimulatedSensor {

   public:

      CEPuck2ColoredBlobPerspectiveCameraDefaultSensor();

      virtual ~CEPuck2ColoredBlobPerspectiveCameraDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual void Enable();

      virtual void Disable();

      /**
       * Returns true if the rays must be shown in the GUI.
       * @return true if the rays must be shown in the GUI.
       */
      inline bool IsShowRays() {
         return m_bShowRays;
      }

      /**
       * Sets whether or not the rays must be shown in the GUI.
       * @param b_show_rays true if the rays must be shown, false otherwise
       */
      inline void SetShowRays(bool b_show_rays) {
         m_bShowRays = b_show_rays;
      }

   protected:

      CEPuck2CameraEquippedEntity*         m_pcCamEntity;
      CControllableEntity*                 m_pcControllableEntity;
      CEmbodiedEntity*                     m_pcEmbodiedEntity;
      CPositionalIndex<CLEDEntity>*        m_pcLEDIndex;
      CPositionalIndex<CEmbodiedEntity>*   m_pcEmbodiedIndex;
      CEPuck2PerspectiveCameraLEDCheckOperation* m_pcOperation;
      bool                                 m_bShowRays;

   };
}

#endif
