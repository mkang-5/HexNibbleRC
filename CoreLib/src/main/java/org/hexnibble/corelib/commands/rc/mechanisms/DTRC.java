package org.hexnibble.corelib.commands.rc.mechanisms;

import org.hexnibble.corelib.commands.rc.RCk;
import org.hexnibble.corelib.motion.DriveController;

public class DTRC extends RCk {
  DriveController driveController;

  public DTRC(DriveController driveController) {
    this.driveController = driveController;
  }

  @Override
  protected void onStartCommand() {
    super.onStartCommand();
    //        driveController.startTrajectory();
  }

  @Override
  protected void processCommand() {
    if (driveController.isIdle()) {
      setCommandStatus(COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED);
    }
  }
}
