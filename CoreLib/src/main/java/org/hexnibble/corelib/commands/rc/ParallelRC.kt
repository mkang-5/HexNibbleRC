package org.hexnibble.corelib.commands.rc

/**
 * Run multiple commands in parallel, ending as soon as a single command completes.
 * @param commandList The list of RCs to run.
 * @param commandID The ID of the command.
 * @see ParallelRaceRC
 */
class ParallelRC @JvmOverloads constructor(
  private var commandList: MutableList<RC>,
  commandID: String = "ParallelRC"
) : RC(commandID) {
  /**
   * Run multiple commands in parallel, ending as soon as a single command completes.
   * @param commandList The RCs to run
   * @param commandID The ID of the command
   * @see ParallelRaceRC
   */
  @JvmOverloads
  constructor(vararg commandList: RC, commandID: String = "ParallelRC") : this(
    mutableListOf(*commandList),
    commandID
  )

  override fun processCommand() {
    // Remove command if complete
    commandList.removeIf { obj: RC -> obj.processRC() }
    if (commandList.isEmpty()) {
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    }
  }
}