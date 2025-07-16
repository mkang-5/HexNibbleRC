package org.hexnibble.corelib.commands.rc

import java.util.function.Consumer

/**
 * Run multiple commands in parallel, ending as soon as a single command completes.
 * @param commandList The list of RCs to run.
 * @param commandID The ID of the command.
 * @see ParallelRC
 */
class ParallelRaceRC @JvmOverloads constructor(
  private var commandList: MutableList<RC>,
  commandID: String = "ParallelRaceRC"
) : RC(commandID) {
  /**
   * Run multiple commands in parallel, ending as soon as a single command completes.
   * @param commandList The RCs to run
   * @param commandID The ID of the command
   * @see ParallelRC
   */
  @JvmOverloads
  constructor(vararg commandList: RC, commandID: String = "ParallelRaceRC") : this(
    mutableListOf(*commandList),
    commandID
  )

  override fun processCommand() {
    // Use removeIf to check each command in the list and remove it if it is complete
    commandList.forEach(
      Consumer { rc: RC ->
        if (rc.processRC()) commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
      })
  }
}