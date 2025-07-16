package org.hexnibble.corelib.commands.rc

/**
 * Null RC. Does nothing.
 */
class NullRC(commandID: String = "NullRC") : RC(commandID) {
  override fun processCommand() {
    commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
  }
}