package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Pose2D;

public class Spin implements CorePath {
   private final double targetIMUHeadingDegrees;
   private final CorePath.ROTATION_DIRECTION rotationDirection;

   public Spin(double targetIMUHeadingDegrees, CorePath.ROTATION_DIRECTION rotationDirection) {
      this.targetIMUHeadingDegrees = targetIMUHeadingDegrees;
      this.rotationDirection = rotationDirection;
   }

   /**
    *
    * @return Target IMU Heading (degrees)
    */
   public double getTargetHeading() {
      return targetIMUHeadingDegrees;
   }

   public CorePath.ROTATION_DIRECTION getRotationDirection() {
      return rotationDirection;
   }

   public double getClosestInterpolatedTValue(Pose2D pose) {
      return 0.0;
   }
}
