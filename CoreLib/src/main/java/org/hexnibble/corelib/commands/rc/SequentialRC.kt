package org.hexnibble.corelib.commands.rc

/**
 * Run multiple commands in sequence, ending when the last command completes.
 * @param commandList The list of RCs to run.
 * @param commandID The ID of the command
 */
class SequentialRC @JvmOverloads constructor(
  private var commandList: MutableList<RC>,
  commandID: String = "SequentialRC"
) : RC(commandID) {
  /**
   * Run multiple commands in sequence, ending when the last command completes.
   * @param commandList The RCs to run
   * @param commandID The ID of the command
   */
  @JvmOverloads
  constructor(vararg commandList: RC, commandID: String = "SequentialRC") : this(
    mutableListOf(*commandList),
    commandID
  )

  override fun processCommand() {
    // Get the current command and run it.
    // If it has completed, then remove the command from the list.
    if (commandList.isEmpty()) {
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    } else {
      if (commandList[0].processRC()) {
        commandList = commandList.subList(1, commandList.size)
      }
      if (commandList.isEmpty()) {
        commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
      }
    }
  }
}