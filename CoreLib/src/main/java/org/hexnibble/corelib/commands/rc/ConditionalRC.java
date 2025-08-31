package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;

public class ConditionalRC extends RC {
   private final BooleanSupplier condition;
   private final RC trueConditionRC;
   private final RC falseConditionRC;
   private boolean conditionResult;

   public ConditionalRC(
         @NonNull BooleanSupplier condition, RC trueConditionRC, RC falseConditionRC) {
      super("ConditionalRC");
      this.condition = condition;
      this.trueConditionRC = trueConditionRC;
      this.falseConditionRC = falseConditionRC;
   }

   public ConditionalRC(@NonNull BooleanSupplier condition, RC trueConditionRC) {
      this(condition, trueConditionRC, new NullRC());
   }

   @Override
   protected void onStartCommand() {
      conditionResult = condition.getAsBoolean();
   }

   @Override
   protected void processCommand() {
      if ((conditionResult ? trueConditionRC : falseConditionRC).processRC())
         setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
   }
}
