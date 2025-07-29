package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;
import java.util.function.BooleanSupplier;

@Deprecated(since = "5/21/25", forRemoval = true)
public class WaitForConditionRCJ extends RCk {
  private final BooleanSupplier condition;
  private final RCk rc;
  private boolean conditionTriggered = false;

  /**
   * This command waits for the specified condition to occur within the specified time limit. If the
   * condition occurs, the specified RC is run. If the condition does not occur within the time
   * limit, nothing happens.
   */
  public WaitForConditionRCJ(@NonNull BooleanSupplier condition, int timeout_ms, RCk rc) {
    super("WaitForConditionCallbackRC", timeout_ms);
    this.condition = condition;
    this.rc = rc;
  }

  public WaitForConditionRCJ(@NonNull BooleanSupplier condition, RCk rc) {
    this(condition, -1, rc);
  }

  /**
   * Wait for a condition. If the condition is met prior to timing out, the runnable function is
   * called at that time. This is different from the version that provides a RC, because that will
   * have been created at the time the WaitForConditionRC is created (including variable values at
   * the time of creation).
   *
   * @param condition
   * @param timeout_ms
   * @param runnableIfConditionMet
   */
  public WaitForConditionRCJ(
      @NonNull BooleanSupplier condition, int timeout_ms, Runnable runnableIfConditionMet) {
    this(condition, timeout_ms, new InstantRCk(runnableIfConditionMet));
  }

  public WaitForConditionRCJ(@NonNull BooleanSupplier condition, Runnable runnableIfConditionMet) {
    this(condition, -1, new InstantRCk(runnableIfConditionMet));
  }

  public WaitForConditionRCJ(@NonNull BooleanSupplier condition, int timeout_ms) {
    this(condition, timeout_ms, new NullRCk());
  }

  public WaitForConditionRCJ(@NonNull BooleanSupplier condition) {
    this(condition, -1, new NullRCk());
  }

  @Override
  protected void processCommand() {
    // If condition has not yet been met, check whether it is met now
    // If the condition does become met, remove the timer so the subsequent action can proceed
    if (!conditionTriggered) {
      conditionTriggered = condition.getAsBoolean();
      if (conditionTriggered) {
        setMaxCommandDurationMs(-1);
        //                Msg.log("WaitForConditionRC", "processCommand", "Condition Triggered");
      }
    }

    // If the condition is met, run the specified action (or wrapped callback)
    if (conditionTriggered && rc.processRC()) {
      setCommandStatus(COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED);
    }
  }

  //    @Override
  //    public boolean processRC() {
  //        ensureCommandStarted();
  //
  //        if (checkIfCommandTimedOut()) return true;
  //        else {
  //            // If the condition is met,
  //            // call the callback and end this command
  //            if (!conditionTriggered) conditionTriggered = condition.getAsBoolean();
  //
  //            if (conditionTriggered) {
  //                return rc.processRC();
  //            }
  //        }
  //        return false;
  //    }
}
