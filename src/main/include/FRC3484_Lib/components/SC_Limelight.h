#ifndef SC_LIMELIGHT_H
#define SC_LIMELIGHT_H

#include "networktables/NetworkTableInstance.h"

namespace SC
{
	enum SC_StreamMode { STANDARD, PIP_MAIN, PIP_SECONDARY };
	enum SC_LEDMode	{ AUTO = 0 , LED_OFF = 1, BLINK = 2, LED_ON = 3 };

	class SC_Limelight
	{
	public:
		SC_Limelight(double Angle, double LensHeight);
		~SC_Limelight();

		bool HasTarget();

		/**
		 * @brief Returns the horizontal offset of target center to image center in deg.
		 * 		  LL1: -27° to +27° | LL2: -29.8° to +29.8°
		 */
		double GetOffsetX();
		
		/**
		 * @brief Returns the vertical offset of target center to image center in deg.
		 * 		  LL1: -20.5° to +20.5° | LL2: -24.85° to +24.85°
		 */
		double GetOffsetY();

		/**
		 * @brief Returns the percentage of the total image area the target occupies
		 */
		double GetTargetArea();

		/**
		 * @brief Returns the skew (rotation) of the target area in the image (-90° to 0°).
		 */
		double GetSkew();

		/**
		 * @brief Returns the pipeline's latency in ms. Add at least 11ms for image capture latency
		 */
		double GetPipelineLatency();

		/**
		 * @brief Returns the length of the shortest side of the bounding box in pixels
		 */
		double GetBBShort();

		/**
		 * @brief Returns the length of the longest side of the bounding box in pixels
		 */
		double GetBBLong();

		/**
		 * @brief Returns the horizontal length of the rough bounding box (0 - 320 pixels)
		 */
		double GetBBWidth();

		/**
		 * @brief Returns the vertical length of the rough bounding box (0 - 320 pixels)
		 */
		double GetBBHeight();

		/**
		 * @brief returns the distance between the camera and target along the horizontal axis
		 */
		double GetDistanceFromTarget();

		int GetActivePipeline();

		void SetLEDMode(SC::SC_LEDMode Mode);
		void SetDriverCam();
		void SetVisionTracking();
		void SetPipeline(int Pipeline);
		void SetStreamMode(SC::SC_StreamMode Mode);

		/**
		 * @brief Sets the angle of the camera relative to the horizontal plane in degrees
		 */
		void SetCameraAngle(double Angle);

		/**
		 * @brief Sets the height of the lens relative to the floor.
		 */
		void SetLensHeight(double Height);

		/**
		 * @brief Sets the height of the vision target relative to the floor.
		 */
		void SetTargetHeight(double Height);

	private:
		std::shared_ptr<nt::NetworkTable> inst;
		double angle, lensHeight, targetHeight;
	};
}

#endif // SC_LIMELIGHT_H