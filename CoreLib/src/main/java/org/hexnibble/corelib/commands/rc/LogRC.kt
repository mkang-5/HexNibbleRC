package org.hexnibble.corelib.commands.rc

import org.hexnibble.corelib.misc.Msg

/**
 * Logs a message.
 * @param logMessage The message to log.
 * @param commandID The ID of the command.
 */
class LogRC @JvmOverloads constructor(private val logMessage: String, commandID: String = "LogRC") :
  RC(commandID, -1) {
  override fun processCommand() {
    Msg.log(javaClass.simpleName, "", logMessage)
    commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
  }
}