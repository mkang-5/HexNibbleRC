package org.hexnibble.corelib.robot;

import org.hexnibble.corelib.misc.Pose2D;

public interface OdometryIface {

   /**
    * Reset odometry wheel encoders and pose
    */
   void resetEncodersAndPose();

   /**
    * Set current pose (alliance-centric CF).
    *
    * @param newPose Alliance-centric X, Y coordinates (mm) and alliance-centric heading (radians)
    */
   void setPoseEstimate(Pose2D newPose);

   /**
    * Update odometry with new encoder values. This function should be called each time through the
    * control loop to keep updating the odometry.
    *
    * @param IMUHeadingDegrees This is only used for 2-wheel odometry.
    *                         It is ignored for 3-wheel and OctoQuad
    */
   void updateOdometry(double IMUHeadingDegrees);

   /**
    * Get the current alliance-centric pose.
    *
    * @return Alliance CF pose. Heading is IMU-style in radians.
    */
   Pose2D getCurrentPose();

   /**
    * Get the current alliance-centric IMU heading.
    *
    * @return Alliance CF IMU heading (degrees).
    */
   double getIMUHeadingDegrees();
}
