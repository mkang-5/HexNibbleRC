package org.hexnibble.corelib.commands.rc;

public class WaitRC extends RC {
   public WaitRC(int waitDuration) {
      super("WaitRC", waitDuration);
   }

   @Override
   protected void processCommand() {
   }
}
