package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;

import org.hexnibble.corelib.commands.rc.deprecated.kotlin.RCk;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Like ConditionalRC but gets the RC during processCommand() instead of in the constructor.
 */
public class JITConditionalRC extends RCk {
  private final BooleanSupplier condition;
  private final Supplier<RCk> trueConditionRC, falseConditionRC;
  private boolean conditionResult;
  private boolean RCObtained;
  private RCk RCtoRun;

  /**
   * Like ConditionalRC but evaluates the condition when the RC is started, instead of in the
   * constructor.
   * @param condition The condition to check.
   * @param trueConditionRC The RC to run if the condition is true.
   * @param falseConditionRC The RC to run if the condition is false.
   */
  public JITConditionalRC(@NonNull BooleanSupplier condition,
      Supplier<RCk> trueConditionRC, Supplier<RCk> falseConditionRC) {

    super("JITConditionalRC");

    this.condition = condition;
    this.trueConditionRC = trueConditionRC;
    this.falseConditionRC = falseConditionRC;
  }

  @Override
  protected void onStartCommand() {
    conditionResult = condition.getAsBoolean();
  }

  @Override
  protected void processCommand() {
    if (!RCObtained) {
      RCObtained = true;
      if (conditionResult) RCtoRun = trueConditionRC.get();
      else RCtoRun = falseConditionRC.get();
    }

    if (RCtoRun.processRC()) {
      setCommandStatus(COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED);
    }
  }
}
