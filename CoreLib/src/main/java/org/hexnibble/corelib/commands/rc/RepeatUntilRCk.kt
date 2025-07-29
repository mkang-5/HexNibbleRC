package org.hexnibble.corelib.commands.rc

import org.hexnibble.corelib.misc.Msg
import java.util.function.BooleanSupplier
import java.util.function.Supplier

/**
 * Repeat a command until a condition is true.
 * @param rcSupplier The command to repeat specified as a supplier.
 * @param condition The condition to check.
 * @param commandID The ID of the command.
 */
class RepeatUntilRCk @JvmOverloads constructor(
    private val rcSupplier: Supplier<RCk>,
    private val condition: BooleanSupplier,
    commandID: String = "RepeatUntilRC"
) : RCk(commandID) {
  private lateinit var command: RCk

  override fun onStartCommand() {
    Msg.log("RepeatUntilRC", "onStartCommand", "Running supplier")
    command = rcSupplier.get()
  }

  override fun processCommand() {
    if (command.processRC()) {
      if (condition.asBoolean) {
        Msg.log("RepeatUntilRC", "", "Condition is true so command successful")
        commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
      } else {
        Msg.log("RepeatUntilRC", "", "Condition is false so resetting the command")
//        command.resetRC();
        command = rcSupplier.get()
      }
    }
  }
}