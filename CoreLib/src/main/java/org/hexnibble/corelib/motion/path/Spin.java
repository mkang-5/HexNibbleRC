package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;

public class Spin extends CorePath {
   private final CorePath.ROTATION_DIRECTION rotationDirection;
   private final double targetHeadingToleranceRadians = Math.toRadians(2.0);

   public Spin(double targetIMUHeadingRadians, CorePath.ROTATION_DIRECTION rotationDirection) {
      super(new Pose2D(0.0, 0.0, targetIMUHeadingRadians));
      this.rotationDirection = rotationDirection;
   }

//   @Override
//   public double getXError(Pose2D currentPose) {
//      return 0.0;
//   }
//
//   @Override
//   public double getYError(Pose2D currentPose) {
//      return 0.0;
//   }

   @Override
   public Pose2D getPoseError(Pose2D currentPose) {
      return new Pose2D(new Vector2D(0.0, 0.0),
            Field.addRadiansToIMUHeading(getTargetPose().heading, -currentPose.heading));
   }

   @Override
   public boolean isPathComplete(Pose2D currentPose) {
      // Only check if a path is complete if it has not already been completed.
      if (isPathComplete) {
         Msg.log(getClass().getSimpleName(), "isPathComplete", "isPathComplete=true");
         return true;
      }
      else {
         Msg.log(getClass().getSimpleName(), "isPathComplete", "currentHdg(deg)=" + Math.toDegrees(currentPose.heading) + ", targetIMUHdg(deg)=" + Math.toDegrees(getTargetHeading()));
         isPathComplete = Math.abs(Field.addRadiansToIMUHeading(currentPose.heading, -getTargetHeading())) < targetHeadingToleranceRadians;
//         if (isPathComplete) {
//            holdPose.x = currentPose.x;
//            holdPose.y = currentPose.y;
//            Msg.log(getClass().getSimpleName(), "isPathComplete", "isPathComplete is newly true so setting holdPose to " + holdPose.toString());
//         }
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
}
