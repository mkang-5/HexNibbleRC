package org.hexnibble.corelib.commands.rc.group;

import org.hexnibble.corelib.commands.rc.RC;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequentialRC extends RC {
   private List<RC> commandList;

   public SequentialRC(List<RC> commandList) {
      super("SequentialRC");
      this.commandList = new ArrayList<>(commandList);
   }

   public SequentialRC(RC... commands) {
      this(Arrays.asList(commands));
   }

   @Override
   protected void processCommand() {
      // Get the current command and run it.
      // If it has completed, then remove the command from the list.
      if (commandList.isEmpty()) {
         setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
      } else {
         if (commandList.get(0).processRC()) {
            commandList = commandList.subList(1, commandList.size());
         }
         if (commandList.isEmpty()) {
            setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
         }
      }
   }
}
