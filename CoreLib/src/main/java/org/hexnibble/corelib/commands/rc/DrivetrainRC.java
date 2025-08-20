package org.hexnibble.corelib.commands.rc;

import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.motion.DriveController;
import org.hexnibble.corelib.motion.path.PathChain;

public class DrivetrainRC extends RC {
   private final PathChain pathChain;
   private final DriveController dtController;

   /**
    * Full constructor. Targets are alliance-centric CF.
    *
    * @param dtController Drivetrain Controller object
    * @param pathChain Path chain for movements
    */
   public DrivetrainRC(DriveController dtController, PathChain pathChain) {
      super("DrivetrainRC");

      this.dtController = dtController;
      this.pathChain = pathChain;
   }

   @Override
   protected void onStartCommand() {
      dtController.startTrajectory(pathChain);
   }

   @Override
   protected void processCommand() {
//      Msg.log(getClass().getSimpleName(), "processCommand", "In DrivetrainRC processCommand");
      if ((dtController.getStatus() == DriveController.STATUS.HOLDING) ||
            (dtController.getStatus() == DriveController.STATUS.IDLE)) {
         Msg.log(getClass().getSimpleName(), "processCommand", "In DrivetrainRC processCommand - dtController.isIdle is true");
         setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
      }
   }
}
