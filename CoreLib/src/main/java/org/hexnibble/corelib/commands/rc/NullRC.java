package org.hexnibble.corelib.commands.rc;

/**
 * Null RC. Does nothing.
 */
public class NullRC extends RC {
   public NullRC() {
      super("NullRC");
   }

   @Override
   protected void processCommand() {
      setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
   }
}
