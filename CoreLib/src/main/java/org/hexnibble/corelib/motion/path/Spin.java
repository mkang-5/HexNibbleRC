package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;

public class Spin implements CorePath {
   private final double targetIMUHeadingRadians;
   private final CorePath.ROTATION_DIRECTION rotationDirection;
   private final double targetHeadingToleranceRadians = Math.toRadians(2.0);

   public Spin(double targetIMUHeadingRadians, CorePath.ROTATION_DIRECTION rotationDirection) {
      this.targetIMUHeadingRadians = targetIMUHeadingRadians;
      this.rotationDirection = rotationDirection;
   }

   /**
    *
    * @return Target IMU Heading (degrees)
    */
   public double getTargetHeading() {
      return targetIMUHeadingRadians;
   }

   public CorePath.ROTATION_DIRECTION getRotationDirection() {
      return rotationDirection;
   }

   @Override
   public boolean isPathComplete(Pose2D currentPose) {
      return Math.abs(Field.addRadiansToIMUHeading(currentPose.heading, - targetIMUHeadingRadians)) < targetHeadingToleranceRadians;
   }

   @Override
   public double getClosestInterpolatedTValue(Pose2D pose) {
      return 0.0;
   }
}
