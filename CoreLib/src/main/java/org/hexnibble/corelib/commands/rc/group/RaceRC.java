package org.hexnibble.corelib.commands.rc.group;

import org.hexnibble.corelib.commands.rc.RC;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Run multiple commands in parallel, ending as soon as a single command completes.
 */
public class RaceRC extends RC {
   private final List<RC> commandList;

   public RaceRC(List<RC> commandList) {
      super("RaceRC");
      this.commandList = new ArrayList<>(commandList);
   }

   public RaceRC(RC... commands) {
      this(Arrays.asList(commands));
   }

   @Override
   protected void processCommand() {
      // Use removeIf to check each command in the list and remove it if it is complete
      commandList.forEach(
         rc -> {
            if (rc.processRC()) setCommandStatus(COMMAND_STATUS.COMMAND_SUCCESSFULLY_COMPLETED);
         });
   }
}
