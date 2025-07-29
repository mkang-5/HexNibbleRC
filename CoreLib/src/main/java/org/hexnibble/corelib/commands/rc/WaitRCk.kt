package org.hexnibble.corelib.commands.rc

/**
 * Wait for a specified duration.
 * @param waitDuration The duration to wait in milliseconds.
 * @param commandID The ID of the command..
 */
class WaitRCk @JvmOverloads constructor(
  waitDuration: Int,
  commandID: String = "WaitRC"
) : RCk(commandID, waitDuration) {
  override fun processCommand() {}
}