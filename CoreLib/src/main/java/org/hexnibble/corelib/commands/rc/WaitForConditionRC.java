package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;
import java.util.function.BooleanSupplier;

public class WaitForConditionRC extends RC {
  private final BooleanSupplier condition;
  private final RC rc;
  private boolean conditionTriggered = false;

  /**
   * This command waits for the specified condition to occur within the specified time limit. If the
   * condition occurs, the specified RC is run. If the condition does not occur within the time
   * limit, nothing happens.
   */
  public WaitForConditionRC(@NonNull BooleanSupplier condition, int timeout_ms, RC rc) {
    super("WaitForConditionRC", timeout_ms);
    this.condition = condition;
    this.rc = rc;
  }

  public WaitForConditionRC(@NonNull BooleanSupplier condition, RC rc) {
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
  public WaitForConditionRC(
      @NonNull BooleanSupplier condition, int timeout_ms, Runnable runnableIfConditionMet) {
    this(condition, timeout_ms, new InstantRC(runnableIfConditionMet));
  }

  public WaitForConditionRC(@NonNull BooleanSupplier condition, Runnable runnableIfConditionMet) {
    this(condition, -1, new InstantRC(runnableIfConditionMet));
  }


  @Override
  protected void processCommand() {
    // The parent RC already checks if command has timed out. If
    // If condition has not yet been met, check whether it is met now
    if (!conditionTriggered) {
      conditionTriggered = condition.getAsBoolean();

      // If the condition does become met, remove the timer so the subsequent action can proceed
      if (conditionTriggered) {
        setMaxCommandDurationMs(-1);
//        Msg.log("WaitForConditionRC", "processCommand", "Condition Triggered");
      }
    }

    // If the condition has been met, run the specified action until it is complete
    if (conditionTriggered && rc.processRC()) {
      setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
    }
  }
}
