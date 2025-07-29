package org.hexnibble.corelib.commands.rc.mechanisms;

import org.hexnibble.corelib.commands.rc.RCk;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Timer;
import org.hexnibble.corelib.robot_system.LinearMechanism;

@Deprecated(since = "5/25/25", forRemoval = true)
public class MoveLinearMechanismToPositionRCJ extends RCk {
  LinearMechanism linearMechanism;
  double targetPosition_mm;
  //    boolean setPowerToZeroWhenRetracted;

  // Use to track whether the encoder offset was reset. We don't need to do this more than once for
  // the duration of this particular command
  private boolean encoderOffsetReset;

  public MoveLinearMechanismToPositionRCJ(
      String commandID,
      LinearMechanism mechanism,
      double targetPosition_mm,
      //                                           boolean setPowerToZeroWhenRetracted,
      int maxCommandDuration_ms) {
    super(commandID, maxCommandDuration_ms);
    this.linearMechanism = mechanism;
    this.targetPosition_mm = targetPosition_mm;
    //        this.setPowerToZeroWhenRetracted = setPowerToZeroWhenRetracted;
    this.encoderOffsetReset = false;
  }

  public MoveLinearMechanismToPositionRCJ(
      String commandID, LinearMechanism mechanism, double targetPosition_mm) {
    this(commandID, mechanism, targetPosition_mm, -1);
  }

  @Override
  protected void onStartCommand() {
    targetPosition_mm = linearMechanism.moveToPosition_mm(targetPosition_mm, 1.0, true);
  }

  @Override
  protected void processCommand() {
    // Status check if this command is taking too long
    if (getElapsedCommandTime(Timer.TimerUnit.ms) > 2500) {
      Msg.log(
          getClass().getSimpleName(),
          "processCommand",
          getCommandID()
              + " taking too long (>2500ms). Current Position="
              + linearMechanism.getCurrentPositionMotor1_mm()
              + ", Target="
              + targetPosition_mm);
    }

    // Only change the encoder offset once per command
    if (!encoderOffsetReset && linearMechanism.isTouchSensorPressed()) {
      Msg.logIfDebug(
          getClass().getSimpleName(),
          "processCommand",
          "Intake touch sensor pressed so setting zero position count offset.");
      linearMechanism.setEncoderZeroPositionCountOffset();

      // If the offset is changed, then the move command must be resent
      targetPosition_mm = linearMechanism.moveToPosition_mm(targetPosition_mm, 1.0, true);
      encoderOffsetReset = true;
    }

    if ((Math.abs(targetPosition_mm - linearMechanism.getCurrentPositionMotor1_mm()) < 10.0)
        || (linearMechanism.getCurrentPositionMotor1_mm() <= -10.0)) {
      // Removed stuff below since this would prevent the PID from continuing to correct to the
      // target position
      //            if (targetSlidePosition <= 10.0) {
      //                system.stopLinearSystem(ITDConstants.IS_LINEAR_SLIDE_NAME,
      // setPowerToZeroWhenRetracted);
      //            }
      //            Msg.log(getClass().getSimpleName(), "processCommand",
      //                    commandID + " returning true");

      setCommandStatus(COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED);
    }
  }

  //    @Override
  //    public boolean processRC() {
  //        ensureCommandStarted();
  //
  //        // Status check if this command is taking too long
  //        if (getElapsedCommandTime(Timer.TimerUnit.ms) > 2500) {
  //            Msg.log(getClass().getSimpleName(), "processCommand",
  //                commandID + " taking too long (>2500ms). Current Position="
  //                        + linearMechanism.getCurrentPositionMotor1_mm()
  //                        + ", Target=" + targetPosition_mm);
  //        }
  //
  //        // Only change the encoder offset once per command
  //        if (!encoderOffsetReset && linearMechanism.isTouchSensorPressed()) {
  //            Msg.logIfDebug(getClass().getSimpleName(), "processCommand", "Intake touch sensor
  // pressed so setting zero position count offset.");
  //            linearMechanism.setEncoderZeroPositionCountOffset();
  //
  //            // If the offset is changed, then the move command must be resent
  //            targetPosition_mm = linearMechanism.moveToPosition_mm(targetPosition_mm, 1.0, true);
  //            encoderOffsetReset = true;
  //        }
  //
  //        if ((Math.abs(targetPosition_mm - linearMechanism.getCurrentPositionMotor1_mm()) < 10.0)
  //                || (linearMechanism.getCurrentPositionMotor1_mm() <= -10.0)) {
  //            // Removed stuff below since this would prevent the PID from continuing to correct
  // to the target position
  ////            if (targetSlidePosition <= 10.0) {
  ////                system.stopLinearSystem(ITDConstants.IS_LINEAR_SLIDE_NAME,
  // setPowerToZeroWhenRetracted);
  ////            }
  //            Msg.log(getClass().getSimpleName(), "processCommand",
  //                    commandID + " returning true");
  //
  //            return true;
  //        }
  //        else {
  //            Msg.log(getClass().getSimpleName(), "processCommand",
  //                    commandID + " returning false");
  //            return false;
  //        }
  //    }
}
