package org.hexnibble.corelib.commands.rc;

public class InstantRC extends RC {
   Runnable function;

   public InstantRC(Runnable function) {
      this.function = function;
   }

   @Override
   protected void processCommand() {
      function.run();
      setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
   }
}
