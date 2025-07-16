package org.hexnibble.corelib.commands.rc;

import org.hexnibble.corelib.misc.Timer;

@Deprecated(since = "5/24/25", forRemoval = true)
public abstract class RCJ {
  protected final String commandID;
  private int maxCommandDuration_ms;
  private Timer commandDurationTimer;

  public enum COMMAND_STATUS {
    COMMAND_CREATED,
    COMMAND_RUNNING,
    COMMAND_FAILED,
    COMMAND_TIMED_OUT,
    COMMAND_SUCCESSFULLY_COMPLETED
  }

  protected COMMAND_STATUS commandStatus;
  protected boolean commandHasStarted;
  protected final boolean logCommandStart;

  // Functions to run on start, failed, and complete
  protected Runnable onStartCallbackFunction, onCompleteCallbackFunction;

  public RCJ(
      String commandID,
      int maxCommandDuration_ms,
      Runnable onStartCallbackFunction,
      Runnable onCompleteCallbackFunction,
      boolean logCommandStart) {
    this.commandID = commandID;
    this.maxCommandDuration_ms = maxCommandDuration_ms < 0 ? -1 : maxCommandDuration_ms;

    this.onStartCallbackFunction = onStartCallbackFunction;
    this.onCompleteCallbackFunction = onCompleteCallbackFunction;

    this.commandStatus = COMMAND_STATUS.COMMAND_CREATED;
    this.commandHasStarted = false;
    this.logCommandStart = logCommandStart;
  }

  public RCJ(
      String commandID,
      int maxCommandDuration_ms,
      Runnable onStartCallbackFunction,
      Runnable onCompleteCallbackFunction) {
    this(
        commandID,
        maxCommandDuration_ms,
        onStartCallbackFunction,
        onCompleteCallbackFunction,
        false);
  }

  public RCJ(String commandID, int maxCommandDuration_ms) {
    this(commandID, maxCommandDuration_ms, null, null);
  }

  public RCJ(String commandID) {
    this(commandID, -1, null, null);
  }

  public RCJ(int maxCommandDuration_ms) {
    this("", maxCommandDuration_ms, null, null);
  }

  public RCJ() {
    this("", -1, null, null);
  }

  protected long getElapsedCommandTime(Timer.TimerUnit timeUnit) {
    return commandDurationTimer.getElapsedTime(timeUnit);
  }

  protected void setMaxCommandDuration_ms(int maxCommandDuration_ms) {
    this.maxCommandDuration_ms = maxCommandDuration_ms;
  }

  protected final void ensureCommandStarted() {
    if (!commandHasStarted) {
      commandHasStarted = true;
      commandStatus = COMMAND_STATUS.COMMAND_RUNNING;

      if (onStartCallbackFunction != null) {
        onStartCallbackFunction.run();
      }

      // Start the timer after the callback function runs
      if (maxCommandDuration_ms >= 0) {
        commandDurationTimer = new Timer();
      }

      onStartCommand();
    }
  }

  protected final void setCommandStatus(COMMAND_STATUS status) {
    commandStatus = status;
  }

  /**
   * If the command has started, check whether the elapsed time has exceeded the maximum duration
   * and set the COMMAND_TIMED_OUT flag if so.
   */
  private void checkIfCommandTimedOut() {
    if (commandHasStarted) {
      // Check if command timed out, if applicable
      if ((maxCommandDuration_ms == 0)
          || ((maxCommandDuration_ms > 0)
              && (commandDurationTimer.getElapsedTime(Timer.TimerUnit.ms)
                  > maxCommandDuration_ms))) {

        commandStatus = COMMAND_STATUS.COMMAND_TIMED_OUT;
      }
    }
  }

  /**
   * Function called when the command is first starting. This is called after any specified
   * onStartCallbackFunction. Override this to create functionality for derived classes.
   */
  protected void onStartCommand() {}

  public final String getCommandID() {
    return commandID;
  }

  /**
   * This function is called each loop and should be overridden by inherited classes to provide the
   * appropriate functionality.
   */
  protected abstract void processCommand();

  /**
   * This is the public function that is called by the RCController each loop to run this command.
   * This should rarely ever be overridden.
   *
   * @return Return whether the command is done. True = done. False = Still active
   */
  public boolean processRC() {
    ensureCommandStarted();
    processCommand();
    checkIfCommandTimedOut();

    return !((commandStatus
            == COMMAND_STATUS
                .COMMAND_CREATED) // If the command were reset, it would be in created status
        || (commandStatus == COMMAND_STATUS.COMMAND_RUNNING));
  }

  /** Used to reset commands to be able to be run again */
  public void resetRC() {
    commandStatus = COMMAND_STATUS.COMMAND_CREATED;
    commandHasStarted = false;
  }

  public void setCommandComplete() {
    commandStatus = COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED;
  }
}
