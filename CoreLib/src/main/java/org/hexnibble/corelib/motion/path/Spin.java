package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;

public class Spin extends CorePath {
   private final double targetIMUHeadingRadians;
   private final CorePath.ROTATION_DIRECTION rotationDirection;
   private final double targetHeadingToleranceRadians = Math.toRadians(2.0);
//   private final Pose2D targetPose;

   public Spin(double targetIMUHeadingRadians, CorePath.ROTATION_DIRECTION rotationDirection) {
      super(new Pose2D(0.0, 0.0, targetIMUHeadingRadians));
      this.targetIMUHeadingRadians = targetIMUHeadingRadians;
//      this.targetPose = new Pose2D(0.0, 0.0, targetIMUHeadingRadians);
      this.rotationDirection = rotationDirection;
   }

   @Override
   public boolean isPathComplete(Pose2D currentPose) {
      // Only check if a path is complete if it has not already been completed.
      if (isPathComplete) {
         Msg.log(getClass().getSimpleName(), "isPathComplete", "isPathComplete=true");
         return true;
      }
      else {
         isPathComplete = Math.abs(Field.addRadiansToIMUHeading(currentPose.heading, - targetIMUHeadingRadians)) < targetHeadingToleranceRadians;
         if (isPathComplete) {
            holdPose.x = currentPose.x;
            holdPose.y = currentPose.y;
            Msg.log(getClass().getSimpleName(), "isPathComplete", "isPathComplete is newly true so setting holdPose to x=" + holdPose.x + ", y=" + holdPose.y + ", hdg(deg)=" + Math.toDegrees(holdPose.heading));
         }
         return isPathComplete;
//         checkPathComplete(currentPose);
      }
   }

//   @Override
//   public Pose2D getTargetPose() {
//      return targetPose;
//   }

   public CorePath.ROTATION_DIRECTION getRotationDirection() {
      return rotationDirection;
   }


   @Override
   public double getClosestInterpolatedTValue(Pose2D pose) {
      return 0.0;
   }
}
