package org.hexnibble.corelib.opmodes;

import org.hexnibble.corelib.commands.rc.RC;
import org.hexnibble.corelib.commands.rc.RCController;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.DriveController;
import org.hexnibble.corelib.robot.CoreRobot;

public class CoreAutoProgram {
  protected CoreRobot robot;
  protected RCController rcController;
  protected DriveController dtController;
  protected final Pose2D startingPose;
  private boolean programComplete;
  protected final Runnable requestOpModeStop;

  public CoreAutoProgram(CoreRobot robot, RCController rcController, Pose2D startingPose,
                         Runnable requestOpModeStop) {
    this.rcController = rcController;
    dtController = robot.drivetrain.getDtController();
    this.startingPose = new Pose2D(startingPose);
    this.programComplete = false;
    this.requestOpModeStop = requestOpModeStop;

    createPaths();
  }

  protected void createPaths() {}

  protected final void q(RC... rc) {
    Msg.log(getClass().getSimpleName(), "q", "Starting q on rcController=" + rcController);
    assert(rcController != null);
    rcController.qRC(rc);
  }

  public final boolean getProgramComplete() {
    return programComplete;
  }

  protected final void setProgramComplete() {
    programComplete = true;
  }
}
