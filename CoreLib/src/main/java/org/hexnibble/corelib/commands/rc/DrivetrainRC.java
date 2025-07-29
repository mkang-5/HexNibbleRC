package org.hexnibble.corelib.commands.rc;

import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.robot.BaseOdometry;

public class DrivetrainRC extends RC {
   public enum ROTATION_DIRECTION {
      CLOCKWISE,
      COUNTERCLOCKWISE
   }

   protected DrivetrainRC.ROTATION_DIRECTION rotationDirection;

   /**
    * Full constructor. Targets are alliance-centric CF.
    *
    * @param commandID String label for this command
    * @param allianceCentricPose Target change in x, y, and IMU-style heading (radians) in
    *     alliance-centric POV. Counter-clockwise turns are positive.
    * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
    * @param odometry Odometry object
    * @param xyPIDSettings PID settings for translation movements
    * @param rotationPIDSettings PID settings for rotation movements
    */
   public DrivetrainRC(
//         Pose2D allianceCentricPose,
//         DrivetrainRC.ROTATION_DIRECTION rotationDirection,
//         BaseOdometry odometry,
//         PIDSettings xyPIDSettings,
//         PIDSettings rotationPIDSettings
   ) {

   }

   @Override
   protected void processCommand() {

   }
}
