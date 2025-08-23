//package org.hexnibble.corelib.commands;
//
//import org.hexnibble.corelib.misc.Timer;
//
///**
// * Base class for drivetrain and other mechanism commands Commands are initialized when they are
// * first added to the queue. Immediately prior to the first iteration of the command, onStart() is
// * called. Immediately after the command is completed, onComplete() is called.
// */
//@Deprecated(since = "5/11/25")
//public abstract class RobotCommand {
//  protected String commandID;
//
//  protected boolean
//      isCommandInitialized; // This flag is set true the very first time initializeCommand() is
//  // called
//  protected boolean hasCommandStarted;
//
//  protected int maxCommandDuration_ms;
//  protected long lapTime;
//  private Timer commandDurationTimer;
//
//  // Functions to run on start, failed, and complete
//  protected Runnable onStartCallbackFunction;
//  protected boolean onStartCallbackFunctionHasRun;
//  protected Runnable onFailedCallbackFunction;
//  protected boolean onFailedCallbackFunctionHasRun;
//  protected Runnable onCompleteCallbackFunction;
//  protected boolean onCompleteCallbackFunctionHasRun;
//
//  protected int currentCommandPhase;
//  protected int phaseStartedStatusBit;
//
//  public enum COMMAND_STATUS {
//    COMMAND_INITIALIZED,
//    COMMAND_RUNNING,
//    COMMAND_FAILED,
//    COMMAND_TIMED_OUT,
//    COMMAND_SUCCESSFULLY_COMPLETED
//  }
//
//  protected COMMAND_STATUS commandStatus;
//
//  protected final int COMMAND_PHASE1 = 0b000000000000001;
//  protected final int STATE_FIND_SAMPLE_WITH_LL = 0b000000000000010;
//  protected final int STATE_MOVE_INTAKE_TOWARD_DETECTED_SAMPLE = 0b000000000000100;
//  protected final int STATE_TILT_ARM_DOWN = 0b000000000001000;
//  protected final int STATE_CLOSE_CLAW = 0b000000000010000;
//  protected final int STATE_RETURN_ARM_TO_NEUTRAL = 0b000000000100000;
//  protected final int STATE_CHECK_FOR_APPROPRIATE_SAMPLE = 0b000000001000000;
//  //    public static int COMMAND_FAILED        = 0b100000000000000;
//  protected final int COMMAND_COMPLETE =
//      0b111111111111111; // This is used for successful completion
//
//  /**
//   * Full Constructor for RobotCommand.
//   *
//   * @param commandID String identifier/name for this command
//   * @param maxCommandDuration_ms Maximum duration allowed for the command (ms). If this value is
//   *     <0, then it will be set to -1 and no maximum duration will be used.
//   */
//  public RobotCommand(
//      String commandID,
//      int maxCommandDuration_ms,
//      Runnable functionToRunOnStart,
//      Runnable functionToRunOnFailed,
//      Runnable functionToRunOnComplete) {
//
//    this.commandID = commandID;
//
//    if (maxCommandDuration_ms < 0) this.maxCommandDuration_ms = -1;
//    else this.maxCommandDuration_ms = maxCommandDuration_ms;
//
//    onStartCallbackFunction = functionToRunOnStart;
//    onFailedCallbackFunction = functionToRunOnFailed;
//    onCompleteCallbackFunction = functionToRunOnComplete;
//
//    hasCommandStarted = false;
//    currentCommandPhase = 0;
//    phaseStartedStatusBit = 0b0;
//  }
//
//  /**
//   * Full Constructor for RobotCommand.
//   *
//   * @param commandID String identifier/name for this command
//   * @param maxCommandDuration_ms Maximum duration allowed for the command (ms). If this value is
//   *     NaN or <0, then it will be set to -1 and no maximum duration will be used.
//   */
//  public RobotCommand(
//      String commandID,
//      int maxCommandDuration_ms,
//      Runnable functionToRunOnStart,
//      Runnable functionToRunOnComplete) {
//    this(commandID, maxCommandDuration_ms, functionToRunOnStart, null, functionToRunOnComplete);
//  }
//
//  /**
//   * Shorter constructor that does not specify functions to run on start or stop
//   *
//   * @param commandID String identifier/name for this command
//   * @param maxCommandDuration_ms Maximum duration allowed for the command (ms). If this value is
//   *     <0, then it will be set to -1 and no maximum duration will be set.
//   */
//  public RobotCommand(String commandID, int maxCommandDuration_ms) {
//    this(commandID, maxCommandDuration_ms, null, null);
//  }
//
//  public RobotCommand() {
//    this("", -1, null, null);
//  }
//
//  public String getCommandID() {
//    return commandID;
//  }
//
//  public COMMAND_STATUS getCommandStatus() {
//    return commandStatus;
//  }
//
//  public int getCurrentCommandPhase() {
//    return currentCommandPhase;
//  }
//
//  /** Used to stop this command prior to it completing. */
//  public void stopCommand() {}
//
//  /**
//   * If the command has been initialized and a maximum duration set, check if the elapsed time has
//   * exceeded the maximum duration.
//   *
//   * @return True/False whether the command has timed out.
//   */
//  protected boolean isCommandTimedOut() {
//    return isCommandInitialized
//        && (maxCommandDuration_ms >= 0)
//        && commandDurationTimer.getElapsedTime(Timer.TimerUnit.ms) > maxCommandDuration_ms;
//  }
//
//  protected long getElapsedCommandTime(Timer.TimerUnit timeUnit) {
//    return commandDurationTimer.getElapsedTime(timeUnit);
//  }
//
//  /** Set a lap time on the command duration timer to use to time a phase */
//  protected void setLapTime() {
//    lapTime = getElapsedCommandTime(Timer.TimerUnit.ms);
//  }
//
//  /**
//   * Calculate the elapse time (in ms) since the lap timer was started.
//   *
//   * @return Elapsed lap time (ms)
//   */
//  protected long getElapsedLapTime_ms() {
//    return getElapsedCommandTime(Timer.TimerUnit.ms) - lapTime;
//  }
//
//  public boolean isCommandInitialized() {
//    return isCommandInitialized;
//  }
//
//  /** This function can be used to initialize things as needed when a command is first created. * */
//  public void initializeCommand() {
//    isCommandInitialized = true;
//    commandStatus = COMMAND_STATUS.COMMAND_INITIALIZED;
//  }
//
//  public boolean hasCommandStarted() {
//    return hasCommandStarted;
//  }
//
//  protected boolean hasPhaseStarted(int phase) {
//    return ((phaseStartedStatusBit & phase) == phase);
//  }
//
//  protected void setPhaseStartedBit(int phase) {
//    phaseStartedStatusBit |= phase;
//  }
//
//  /**
//   * Function called once when the command is first starting. The command duration timer is started
//   * and if there is an onStart callback function, it is called. The processCommand() function will
//   * then be called immediately afterwards in the same loop iteration.
//   */
//  public void onStart() {
//    hasCommandStarted = true;
//    commandStatus = COMMAND_STATUS.COMMAND_RUNNING;
//
//    // Start command duration timer
//    if (commandDurationTimer == null) commandDurationTimer = new Timer();
//    else commandDurationTimer.restartTimer();
//
//    if (onStartCallbackFunction != null) {
//      onStartCallbackFunction.run();
//      onStartCallbackFunctionHasRun = true;
//    }
//  }
//
//  /**
//   * Function called once immediately after the command is complete (in the same loop iteration).
//   */
//  public void onFailed() {
//    commandStatus = COMMAND_STATUS.COMMAND_FAILED;
//    if (onFailedCallbackFunction != null) {
//      onFailedCallbackFunction.run();
//      onFailedCallbackFunctionHasRun = true;
//    }
//  }
//
//  /**
//   * Function called once immediately after the command is complete (in the same loop iteration).
//   */
//  public void onComplete() {
//    commandStatus = COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED;
//
//    if (onCompleteCallbackFunction != null) {
//      onCompleteCallbackFunction.run();
//      onCompleteCallbackFunctionHasRun = true;
//    }
//  }
//
//  public boolean isComplete() {
//    return (commandStatus == COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED)
//        || (currentCommandPhase == COMMAND_COMPLETE);
//  }
//
//  /**
//   * @return Return whether the command is still in process. True = Still processing. False =
//   *     Complete
//   */
//  public abstract boolean processCommand();
//}
