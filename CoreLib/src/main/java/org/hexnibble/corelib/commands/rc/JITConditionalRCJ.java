package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * @deprecated Use JITConditionalRC instead
 */
@Deprecated(since = "5/22/25", forRemoval = true)
public class JITConditionalRCJ extends RCk {
  private final BooleanSupplier condition;
  private final Supplier<RCk> trueConditionRC, falseConditionRC;
  private boolean conditionResult;
  private boolean RCObtained;
  private RCk RCtoRun;

  public JITConditionalRCJ(
      @NonNull BooleanSupplier condition,
      Supplier<RCk> trueConditionRC,
      Supplier<RCk> falseConditionRC) {
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
