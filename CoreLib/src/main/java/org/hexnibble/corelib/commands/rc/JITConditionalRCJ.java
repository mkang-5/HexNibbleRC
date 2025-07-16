package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * @deprecated Use JITConditionalRC instead
 */
@Deprecated(since = "5/22/25", forRemoval = true)
public class JITConditionalRCJ extends RC {
  private final BooleanSupplier condition;
  private final Supplier<RC> trueConditionRC, falseConditionRC;
  private boolean conditionResult;
  private boolean RCObtained;
  private RC RCtoRun;

  public JITConditionalRCJ(
      @NonNull BooleanSupplier condition,
      Supplier<RC> trueConditionRC,
      Supplier<RC> falseConditionRC) {
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
