package org.hexnibble.corelib.commands.rc

/**
 * Executes a Runnable and immediately return.
 * @param function The Runnable to run.
 * @param commandID The ID of the command.
 */
class InstantRC @JvmOverloads constructor(
  private val function: Runnable,
  commandID: String = "InstantRC"
) : RC(commandID) {
  override fun processCommand() {
    function.run()
    commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
  }
}
