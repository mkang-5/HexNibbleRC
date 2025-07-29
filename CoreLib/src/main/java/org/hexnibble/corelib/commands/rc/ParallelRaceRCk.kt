package org.hexnibble.corelib.commands.rc

import java.util.function.Consumer

/**
 * Run multiple commands in parallel, ending as soon as a single command completes.
 * @param commandList The list of RCs to run.
 * @param commandID The ID of the command.
 * @see ParallelRCk
 */
class ParallelRaceRCk @JvmOverloads constructor(
    private var commandList: MutableList<RCk>,
    commandID: String = "ParallelRaceRC"
) : RCk(commandID) {
  /**
   * Run multiple commands in parallel, ending as soon as a single command completes.
   * @param commandList The RCs to run
   * @param commandID The ID of the command
   * @see ParallelRCk
   */
  @JvmOverloads
  constructor(vararg commandList: RCk, commandID: String = "ParallelRaceRC") : this(
    mutableListOf(*commandList),
    commandID
  )

  override fun processCommand() {
    // Use removeIf to check each command in the list and remove it if it is complete
    commandList.forEach(
      Consumer { rc: RCk ->
        if (rc.processRC()) commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
      })
  }
}