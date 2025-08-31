package org.hexnibble.corelib.commands.rc.mechanisms

import org.hexnibble.corelib.commands.rc.deprecated.kotlin.RCk
import org.hexnibble.corelib.misc.Msg
import org.hexnibble.corelib.misc.Timer
import org.hexnibble.corelib.robot_system.LinearMechanism
import kotlin.math.abs

/**
 * @param mechanism The linear mechanism to move.
 * @param targetPositionMm The target position to send the linear mechanism to.
 * @param maxCommandDuration The maximum amount of time before the command times out. Defaults to -1 (never timeout)
 * @param commandID The ID of the command.
 */
class MoveLinearMechanismToPositionRCk @JvmOverloads constructor(
  private val mechanism: LinearMechanism,
  private var targetPositionMm: Double,
  maxCommandDuration: Int = -1,
  commandID: String = "MoveLinearMechanismToPositionRC"
) : RCk(commandID, maxCommandDuration) {
  //    boolean setPowerToZeroWhenRetracted;
  // Use to track whether the encoder offset was reset. We don't need to do this more than once for
  // the duration of this particular command
  private var encoderOffsetReset = false

  override fun onStartCommand() {
    targetPositionMm = mechanism.moveToPosition_mm(targetPositionMm, 1.0, true)
  }

  override fun processCommand() {
    // Status check if this command is taking too long
    if (getElapsedCommandTime(Timer.TimerUnit.ms) > 2500) {
      Msg.log(
        javaClass.getSimpleName(),
        "processCommand",
        commandID
            + " taking too long (>2500ms). Current Position="
            + mechanism.currentPositionMotor1_mm
            + ", Target="
            + targetPositionMm
      )
    }

    // Only change the encoder offset once per command
    if (!encoderOffsetReset && mechanism.isTouchSensorPressed) {
      Msg.logIfDebug(
        javaClass.getSimpleName(),
        "processCommand",
        "Intake touch sensor pressed so setting zero position count offset."
      )
      mechanism.setEncoderZeroPositionCountOffset()

      // If the offset is changed, then the move command must be resent
      targetPositionMm = mechanism.moveToPosition_mm(targetPositionMm, 1.0, true)
      encoderOffsetReset = true
    }

    if ((abs(targetPositionMm - mechanism.currentPositionMotor1_mm) < 10.0)
      || (mechanism.currentPositionMotor1_mm <= -10.0)
    ) {
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    }
  }
}