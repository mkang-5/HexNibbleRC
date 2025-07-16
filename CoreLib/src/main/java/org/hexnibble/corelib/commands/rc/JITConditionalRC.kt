package org.hexnibble.corelib.commands.rc

import java.util.function.BooleanSupplier
import java.util.function.Supplier

/**
 * Like ConditionalRC but gets the RC during processCommand() instead of in the constructor.
 * @param condition The condition to check.
 * @param trueConditionRC The RC to run if the condition is true.
 * @param falseConditionRC The RC to run if the condition is false.
 * @param commandID The ID of the command.
 * @see ConditionalRC
 */
class JITConditionalRC @JvmOverloads constructor(
  private var condition: BooleanSupplier,
  private var trueConditionRC: Supplier<RC>,
  private var falseConditionRC: Supplier<RC>,
  commandID: String = "JITConditionalRC"
) : RC(commandID) {
  private var conditionResult = false
  private var rcObtained = false
  private lateinit var rctoRun: RC

  override fun onStartCommand() {
    // Check the state of the condition and set conditionResult
    conditionResult = condition.asBoolean
  }

  override fun processCommand() {
    if (!rcObtained) {
      rcObtained = true
      rctoRun = if (conditionResult) {
        trueConditionRC.get()
      } else {
        falseConditionRC.get()
      }
    }

    if (rctoRun.processRC()) {
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    }
  }
}