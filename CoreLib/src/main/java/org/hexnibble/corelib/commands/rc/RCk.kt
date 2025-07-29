package org.hexnibble.corelib.commands.rc

import org.hexnibble.corelib.misc.Msg
import org.hexnibble.corelib.misc.Timer
import org.hexnibble.corelib.misc.Timer.TimerUnit

abstract class RCk @JvmOverloads constructor(
  protected val commandID: String = "",
  private var maxCommandDurationMs: Int = -1,
  private var onStartCallbackFunction: Runnable? = null,
  private var onCompleteCallbackFunction: Runnable? = null,
  private val logCommandStart: Boolean = false
) {
  private var commandDurationTimer: Timer? = null

  protected enum class COMMAND_STATUSES {
    COMMAND_CREATED,
    COMMAND_RUNNING,
    COMMAND_FAILED,
    COMMAND_TIMED_OUT,
    COMMAND_SUCCESSFULLY_COMPLETED
  }

  protected var commandStatus: COMMAND_STATUSES = COMMAND_STATUSES.COMMAND_CREATED
  protected var commandHasStarted: Boolean = false

  /**
   * Return elapsed command time.
   * If the command has not yet started, -999 is returned
   */
  protected fun getElapsedCommandTime(timeUnit: TimerUnit): Long {
    return commandDurationTimer?.getElapsedTime(timeUnit) ?: -999
  }

  protected fun setMaxCommandDurationMs(maxCommandDurationMs: Int) {
    this.maxCommandDurationMs = maxCommandDurationMs
  }

  protected fun ensureCommandStarted() {
    if (!commandHasStarted) {
      commandHasStarted = true
      commandStatus = COMMAND_STATUSES.COMMAND_RUNNING
      if (logCommandStart) {
        Msg.log("RC.kt", "ensureCommandStarted()", "Command Started")
      }

      onStartCallbackFunction?.run()

      // Start the timer after the callback function runs
      if (maxCommandDurationMs >= 0) {
        commandDurationTimer = Timer()
      }

      onStartCommand()
    }
  }

  /**
   * If the command has started, check whether the elapsed time has exceeded the maximum duration
   * and set the COMMAND_TIMED_OUT flag if so.
   */
  private fun checkIfCommandTimedOut() {
    if (commandHasStarted) {
      // Check if command timed out, if applicable
      if ((maxCommandDurationMs == 0)
        || ((maxCommandDurationMs > 0)
            && (commandDurationTimer!!.getElapsedTime(TimerUnit.ms)
            > maxCommandDurationMs))
      ) {
        commandStatus = COMMAND_STATUSES.COMMAND_TIMED_OUT
      }
    }
  }

  /**
   * Function called when the command is first starting. This is called after any specified
   * onStartCallbackFunction. Override this to create functionality for derived classes.
   */
  protected open fun onStartCommand() {}

//  fun getCommandID(): String {
//    return commandID
//  }

  /**
   * This function is called each loop and should be overridden by inherited classes to provide the
   * appropriate functionality.
   */
  protected abstract fun processCommand()

  /**
   * This is the public function that is called by the RCController each loop to run this command.
   * This should rarely ever be overridden.
   *
   * @return Return whether the command is done. True = done. False = Still active
   */
  fun processRC(): Boolean {
    ensureCommandStarted()
    processCommand()
    checkIfCommandTimedOut()

    if (!((commandStatus == COMMAND_STATUSES.COMMAND_CREATED) // If the command were reset, it would be in created status
          || (commandStatus == COMMAND_STATUSES.COMMAND_RUNNING))) {
      onCompleteCallbackFunction?.run()
      return true
    } else return false
  }

  /** Used to reset commands to be able to be run again  */
  fun resetRC() {
    commandStatus = COMMAND_STATUSES.COMMAND_CREATED
    commandHasStarted = false
  }

  fun setCommandComplete() {
    commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
  }
}