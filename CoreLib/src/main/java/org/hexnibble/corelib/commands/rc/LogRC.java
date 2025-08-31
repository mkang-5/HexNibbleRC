package org.hexnibble.corelib.commands.rc;

import org.hexnibble.corelib.misc.Msg;

public class LogRC extends RC {
   private final String logMessage;

   public LogRC(String logMessage) {
      super("LogRC", -1);
      this.logMessage = logMessage;
   }

   @Override
   protected void processCommand() {
      Msg.log(getClass().getSimpleName(), "", logMessage);
      setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
   }
}
