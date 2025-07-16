//package org.hexnibble.corelib.opmodes;
//
//import android.util.Log;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import org.hexnibble.corelib.commands.rc.RCController;
//import org.hexnibble.corelib.misc.ConfigFile;
//import org.hexnibble.corelib.misc.Constants;
//import org.hexnibble.corelib.misc.Timer;
//
//@TeleOp(name = "Basic Linear TeleOp with Mecanum Drivetrain", group = "Linear OpMode")
//
//// Remove or comment out the @Disabled line to display this OpMode in the Driver Station OpMode list
//@Disabled
//public class CoreTeleOpMode extends CoreLinearOpMode {
//  private int lastElapsedTime_s = 0;
//
//  public CoreTeleOpMode(ConfigFile configFile, Constants constants) {
//    super(OP_MODE_TYPE.TELE_OP, configFile, constants);
//    Log.i(
//        TAG,
//        "CoreTeleOpMode constructor - Completed parent constructor. Continuing local constructor.");
//
//    Log.i(TAG, "CoreTeleOpMode constructor - Building rumble effects.");
//
//  }
//
//  //    protected void processController1() {
//  //        boolean isOptionShiftDown = controller1.isOptionShiftPressed();
//  //
//  //        // 0b1234
//  //        // 1 = Left Stick X
//  //        // 2 = Left Stick Y
//  //        // 3 = Left or Right Triggers (manual spin turns)
//  //        byte sendNewDrivetrainCommand = 0b0000;
//  /// *
//  //        // Toggle off alliance-centric coordinate if allowed
//  //        if (ConfigFile.ALLOW_ALLIANCE_CENTRIC_DRIVING_OFF) {
//  //            if (controller1.processButton("UseAllianceCentricCoordinates") ==
//  // ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED)
//  //            {
//  //                // Reset the affected driving variables (left stick) since the behavior must
//  // change if this button was pressed.
//  //                prevGamepad1LeftStickX = 0;
//  //                prevGamepad1LeftStickY = 0;
//  //
//  // robot.useAllianceCentricCoordinates(controller1.getButton("UseAllianceCentricCoordinates").getToggleState());
//  //            }
//  //        }
//  // */
//  /// *
//  //        // Process speed mode button
//  //        // If the slow mode button is being held down, then remain in eco mode
//  //        ButtonWrapper.BUTTON_STATE squareButtonState =
//  // controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.square);
//  //        if (squareButtonState == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//  //            driveMode = DRIVE_MODE.ECO;
//  //            sendNewDrivetrainCommand = 0b1111;
//  //        }
//  //        else if (squareButtonState == ButtonWrapper.BUTTON_STATE.NEWLY_RELEASED) {
//  //            driveMode = DRIVE_MODE.REGULAR;
//  //            sendNewDrivetrainCommand = 0b1111;
//  //        }
//  // */
//  /// *
//  //        // Process clear drive command queue button
//  //        if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.right_stick_button) ==
//  // ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//  //            robot.cancelCurrentDrivetrainCommand();
//  //        }
//  // */
//  //
//  /// *
//  //        // Queued 45 degree spin turns
//  //        if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.left_bumper) ==
//  // ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//  //            robot.qDrivetrainSpinTurnToNearest45("CCW_Turn",
//  // DrivetrainRC.ROTATION_DIRECTION.COUNTERCLOCKWISE, 5000);
//  //        }
//  //        else if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.right_bumper) ==
//  // ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//  //            robot.qDrivetrainSpinTurnToNearest45("CW_Turn",
//  // DrivetrainRC.ROTATION_DIRECTION.CLOCKWISE, 5000);
//  //        }
//  // */
//  //        // Manual Drivetrain Movements - Translation and spin movements are independently
//  // controlled.
//  //        // Manual movements will override preset movements
//  //        // Mecanum Drive with gamepad1 left stick
//  //        float filtered_left_stick_x = controller1.getLeftStickX();                    // Obtain
//  // current left stick X value (with dead zone applied).
//  //        if (filtered_left_stick_x != prevGamepad1LeftStickX)                          // Check
//  // if there has been a change in the stick value
//  //        {
//  //            if ((filtered_left_stick_x == 0)
//  //         // If the change is to zero or it is greater than the specified threshold
//  //                    || (Math.abs(filtered_left_stick_x - prevGamepad1LeftStickX) >
//  // Constants.CONTROLLER_STICK_MOVEMENT_THRESHOLD))
//  ////                    || (Math.abs(filtered_left_stick_x - prevGamepad1LeftStickX) >
//  // ConfigFile.CONTROLLER_STICK_MOVEMENT_THRESHOLD))
//  //            {
//  //                sendNewDrivetrainCommand |= 0b1000;
//  //                prevGamepad1LeftStickX = filtered_left_stick_x;
//  //            }
//  //        }
//  //
//  //        float filtered_left_stick_y = controller1.getLeftStickY();                    // Obtain
//  // current left stick Y value (with dead zone applied).
//  //        if (filtered_left_stick_y != prevGamepad1LeftStickY)
//  //       // Check if there has been a change in the stick value
//  //        {
//  //            if ((filtered_left_stick_y == 0)
//  //         // If the change is to zero or it is greater than the specified threshold
//  //                    || (Math.abs(filtered_left_stick_y - prevGamepad1LeftStickY) >
//  // Constants.CONTROLLER_STICK_MOVEMENT_THRESHOLD))
//  ////                    || (Math.abs(filtered_left_stick_y - prevGamepad1LeftStickY) >
//  // ConfigFile.CONTROLLER_STICK_MOVEMENT_THRESHOLD))
//  //            {
//  //                sendNewDrivetrainCommand |= 0b0100;
//  //                prevGamepad1LeftStickY = filtered_left_stick_y;
//  //            }
//  //        }
//  //
//  //        // Left/right trigger buttons are used for manual spin turns.
//  //        // But only if shift buttons is not also pressed
//  //        float spin = previousSpin;
//  //
//  //        if (!isOptionShiftDown) {
//  //            float left_trigger = controller1.getLeftTrigger();
//  //            float right_trigger = controller1.getRightTrigger();
//  //            if (left_trigger != prevGamepad1LeftTrigger)         // Add manual left/CCW spin
//  //            {
//  //                sendNewDrivetrainCommand |= 0b0010;
//  //                spin = left_trigger;
//  //                previousSpin = spin;
//  //
//  //                // Reset stored data if left trigger is released. This allows changes in other
//  // buttons/sticks to be recognized.
//  //                if (left_trigger == 0.0f) {
//  //                    prevGamepad1RightTrigger = 0.0f;
//  //                }
//  //                prevGamepad1LeftTrigger = left_trigger;
//  //            }
//  //            else if ((right_trigger != prevGamepad1RightTrigger) && (left_trigger == 0))   //
//  // Add manual right/CW spin, but only if the left trigger is not already sending a spin command
//  //            {
//  //                sendNewDrivetrainCommand |= 0b0010;
//  //                spin = -right_trigger;
//  //                previousSpin = spin;
//  //
//  //                if (right_trigger == 0.0f) {
//  //                    prevGamepad1LeftTrigger = 0.0f;
//  //                }
//  //                prevGamepad1RightTrigger = right_trigger;
//  //            }
//  //        }
//  //
//  //        if (sendNewDrivetrainCommand != 0) {
//  //            double speedMultiplicationFactor = switch (driveMode) {
//  //                case ECO -> ConfigFile.DRIVETRAIN_ECO_MODE_SPEED_MULTIPLICATION_FACTOR;
//  //                case REGULAR -> ConfigFile.DRIVETRAIN_REGULAR_MODE_SPEED_MULTIPLICATION_FACTOR;
//  //            };
//  //
//  //            if ((sendNewDrivetrainCommand & 0b1000) != 0) {
//  //                robot.setDrivetrainManualMovement_X((double) filtered_left_stick_x *
//  // speedMultiplicationFactor);
//  //            }
//  //            if ((sendNewDrivetrainCommand & 0b0100) != 0) {
//  //                robot.setDrivetrainManualMovement_Y((double) -filtered_left_stick_y *
//  // speedMultiplicationFactor);
//  //            }
//  //            if ((sendNewDrivetrainCommand & 0b0010) != 0) {
//  //                if (ConfigFile.DRIVETRAIN_REGULAR_MODE_SPIN_MULTIPLICATION_FACTOR != 1) {
//  //                    robot.setDrivetrainManualMovement_Spin((double) spin *
//  // ConfigFile.DRIVETRAIN_REGULAR_MODE_SPIN_MULTIPLICATION_FACTOR);
//  //                }
//  //                else {
//  //                    robot.setDrivetrainManualMovement_Spin((double) spin);
//  //                }
//  //            }
//  //        }
//  //    }
//
//
//  @Override
//  public void runOpMode() {
//    createControllersForTeleOp();
////    rcController = new RCController(robot, pedroFollower, opModeType, controller1, controller2);
//
//    displayTelemetryMessageAfterInitialization();
//
//    /////// Wait for the game to start (driver presses PLAY) ///////
//    /////////////////////////////// Note that pressing STOP still continues processing subsequent
//    // statements /////////
//    /////////////////////////////// This can cause problems if any of the subsequent statements try
//    // to use uninitialized variables!! (like at AZCMP 2025)
//    waitForStart();
//
//    // Ensure OpMode was not stopped before running onStartOpMode
//    if (opModeIsActive()) {
//      onPressPlay();
//      while (opModeIsActive()) // Run until the end of the match (driver presses STOP)
//      {
//        rcController.processCommands();
//
//        rumbleControllersForElapsedTimeWarning();
//        processLoopTime();
//        createTelemetryMessageForEachLoop();
//      }
//    } else {
//      Log.i(TAG, "OpMode not active. STOP must have been pressed before OpMode began.");
//    }
//    onStopOpMode();
//  }
//}
