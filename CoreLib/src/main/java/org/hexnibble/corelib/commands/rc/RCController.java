package org.hexnibble.corelib.commands.rc;

import androidx.annotation.NonNull;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hexnibble.corelib.motion.DriveController;
import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.wrappers.controller.ControllerWrapper;

public final class RCController {
   private final CoreRobot robot;
   private final CoreLinearOpMode.OP_MODE_TYPE opModeType;
   private final ControllerWrapper controller1;
   private final ControllerWrapper controller2;
   private final DriveController dtController;
   private final List<RC> activeRCList = new ArrayList<>();

   public RCController(CoreLinearOpMode.OP_MODE_TYPE opModeType,
                       @NonNull CoreRobot robot, @NonNull DriveController dtController,
                       ControllerWrapper controller1, ControllerWrapper controller2) {

      this.robot = robot;
      this.opModeType = opModeType;
      this.controller1 = controller1;
      this.controller2 = controller2;
      this.dtController = dtController;
   }

   public void qRC(RC... rc) {
      activeRCList.addAll(Arrays.asList(rc));
   }

   public void processCommands() {
      // Update the gamepad controllers and run any registered button commands
      if (opModeType == CoreLinearOpMode.OP_MODE_TYPE.TELE_OP) {
         controller1.updateGamepadData();
         controller2.updateGamepadData();
      }

      // Process robot commands
      // This will bulk read hubs and read IMU
      // The robot also processes commands for each robot system
      robot.processCommands();

      // Update general running commands
      if (!activeRCList.isEmpty() && activeRCList.get(0).processRC()) {
         activeRCList.remove(0);
      }

      dtController.processPath(robot.getStoredIMUHeadingDegrees());

//      if (pedroFollower != null) {
//         pedroFollower.update();
//         //            Msg.log("PPose: " + pedroFollower.getPose().getX() + ", " +
//         // pedroFollower.getPose().getY() + ", " +
//         // Math.toDegrees(pedroFollower.getPose().getHeading()));
//      }
   }
}