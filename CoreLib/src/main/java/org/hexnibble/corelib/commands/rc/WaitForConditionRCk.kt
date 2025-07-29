package org.hexnibble.corelib.commands.rc

import org.hexnibble.corelib.misc.Msg
import org.hexnibble.corelib.misc.Timer
import java.util.function.BooleanSupplier

/**
 * @param condition The condition to wait for.
 * @param timeout Specifies max command length in milliseconds.
 * @param rc The robot command to run if the condition is met.
 * @param commandID The ID of the command.
 */
class WaitForConditionRCk @JvmOverloads constructor(
  private val condition: BooleanSupplier,
  private var timeout: Int = -1,
  private var rc: RCk = NullRCk(),
  commandID: String = "WaitForConditionRC"
) : RCk(commandID) {
  private var conditionTriggered = false
  private lateinit var timer: Timer

  override fun onStartCommand() {
    timer = Timer()
  }

  override fun processCommand() {
    Msg.log("Elapsed Time: ${timer.getElapsedTime(Timer.TimerUnit.ms)}")
    // If WaitForConditionRC can timeout (timeout not equal to -1) and WaitForConditionRC has timed
    // out, end WaitForConditionRC. Otherwise, check if condition met
    if (timeout != -1 && timer.getElapsedTime(Timer.TimerUnit.ms) >= timeout) {
      Msg.log("Command Timed Out")
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    } else {
      // If condition has not yet been met, check whether it is met now
      // If the condition does become met, remove the timer so the subsequent action can proceed
      if (!conditionTriggered) {
        Msg.log("Condition not met in initial check")
        conditionTriggered = condition.asBoolean
        Msg.log("Condition Triggered: ${condition.asBoolean}")
        // If condition just triggered, disable timeout and run condition met RC
        if (conditionTriggered) {
          Msg.log("Timeout disabled bc condition triggered")
          timeout = -1
        }
      }

      // If the condition is met, run the specified action (or wrapped callback)
      if (conditionTriggered && rc.processRC()) {
        Msg.log("Condition met and RC finished. Command now complete")
        commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
      }
    }
  }
}