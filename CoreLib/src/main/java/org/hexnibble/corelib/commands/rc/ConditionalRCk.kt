package org.hexnibble.corelib.commands.rc

import java.util.function.BooleanSupplier

/**
 * Run RC based on a boolean condition.
 * @param condition The condition to check.
 * @param trueConditionRC The RC to run if the condition is true.
 * @param falseConditionRC The RC to run if the condition is false.
 * @param commandID The ID of the command.
 * @see JITConditionalRCk
 */
class ConditionalRCk @JvmOverloads constructor(
  private var condition: BooleanSupplier,
  private var trueConditionRC: RCk,
  private var falseConditionRC: RCk = NullRCk(),
  commandID: String = "ConditionalRC"
) : RCk(commandID) {
  private var conditionResult = false

  override fun onStartCommand() {
    // Check the state of the condition and set conditionResult
    conditionResult = condition.asBoolean
  }

  override fun processCommand() {
    // If condition is true, run trueConditionRC. Otherwise, run falseConditionRC
    if (conditionResult) {
      if (trueConditionRC.processRC()) {
        commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
      }
    } else if (falseConditionRC.processRC()) {
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    }
  }
}