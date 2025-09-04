package org.hexnibble.corelib.commands.rc.group;

import org.hexnibble.corelib.commands.rc.RC;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Run multiple commands in parallel, ending only when all commands are completed.
 */
public class ParallelRC extends RC {
   private final List<RC> commandList;

   public ParallelRC(List<RC> commandList) {
      super("ParallelRC");
      this.commandList = new ArrayList<>(commandList);
   }

   public ParallelRC(RC... commands) {
      this(Arrays.asList(commands));
   }

   @Override
   protected void processCommand() {
      // removeIf will cycle through each command in the List and remove it if it is complete
      commandList.removeIf(RC::processRC);
      if (commandList.isEmpty()) {
         setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
      }
   }
}
