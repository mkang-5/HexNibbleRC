package org.hexnibble.corelib.commands.rc

/**
 * Run multiple commands in parallel, ending as soon as a single command completes.
 * @param commandList The list of RCs to run.
 * @param commandID The ID of the command.
 * @see ParallelRaceRCk
 */
class ParallelRCk @JvmOverloads constructor(
    private var commandList: MutableList<RCk>,
    commandID: String = "ParallelRC"
) : RCk(commandID) {
  /**
   * Run multiple commands in parallel, ending as soon as a single command completes.
   * @param commandList The RCs to run
   * @param commandID The ID of the command
   * @see ParallelRaceRCk
   */
  @JvmOverloads
  constructor(vararg commandList: RCk, commandID: String = "ParallelRC") : this(
    mutableListOf(*commandList),
    commandID
  )

  override fun processCommand() {
    // Remove command if complete
    commandList.removeIf { obj: RCk -> obj.processRC() }
    if (commandList.isEmpty()) {
      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
    }
  }
}