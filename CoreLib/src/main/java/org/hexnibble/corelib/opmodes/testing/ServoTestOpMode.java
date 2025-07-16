//package org.hexnibble.corelib.opmodes.testing;
//
//import android.util.Log;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.hexnibble.corelib.misc.Field;
//import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
//import org.hexnibble.corelib.wrappers.controller.ButtonWrapper;
//import org.hexnibble.corelib.wrappers.controller.ControllerWrapper;
//
//@TeleOp(name = "Servo Test Op Mode", group = "Test OpModes")
//
//// Remove or comment out the @Disabled line to display this OpMode in the Driver Station OpMode list
//@Disabled
//public class ServoTestOpMode extends CoreLinearOpMode {
//  // Controller 1
//  private final float prevGamepad1LeftStickX = 0.0f;
//  private final float prevGamepad1LeftStickY = 0.0f;
//  private final float prevGamepad1LeftTrigger = 0.0f;
//  private final float prevGamepad1RightTrigger = 0.0f;
//  private final float previousSpin = 0.0f;
//
//  // Controller 2
//  private final float prevGamepad2LeftStickX = 0.0f;
//  private final float prevGamepad2LeftStickY = 0.0f;
//
//  private final int lastElapsedTime_s = 0;
//
//  private String[] servoNameArray;
//  private int testingServoIndex = 0;
//
//  public ServoTestOpMode() {
//    super(OP_MODE_TYPE.TELE_OP);
//    Log.i(
//        TAG,
//        "CoreTeleOpMode constructor - Completed parent constructor. Continuing local constructor.");
//  }
//
//  /** Process controller 2 buttons */
//  protected void processController2() {
//    controller2.updateGamepadData();
//
//    // Set servo to max position
//    if (controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.triangle)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      robot.setServoToMaxPosition(null, servoNameArray[testingServoIndex]);
//    }
//    // Set servo to min position
//    if (controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.cross)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      robot.setServoToMinPosition(null, servoNameArray[testingServoIndex]);
//    }
//
//    // Increment servo direction down
//    if ((controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_left)
//            == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED)
//        || (controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_down)
//            == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED)) {
//      robot.moveServoPositionDown(null, servoNameArray[testingServoIndex]);
//    }
//    // Increment servo direction up
//    if ((controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_up)
//            == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED)
//        || (controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_right)
//            == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED)) {
//      robot.moveServoPositionUp(null, servoNameArray[testingServoIndex]);
//    }
//
//    if (controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.left_bumper)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      testingServoIndex =
//          (testingServoIndex > 0) ? (testingServoIndex - 1) : (servoNameArray.length - 1);
//    } else if (controller2.getButtonStatus(ControllerWrapper.BUTTON_NAME.right_bumper)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      testingServoIndex =
//          (testingServoIndex < (servoNameArray.length - 1)) ? (testingServoIndex + 1) : 0;
//    }
//  }
//
//  @Override
//  public void runOpMode() {
//    initializeOpMode();
//    servoNameArray = robot.getServoNames();
//
//    telemetry.addLine("READY.");
//    telemetry.update();
//    Log.i(TAG, "FIELD_WIDTH_MM = " + Field.FIELD_WIDTH_MM);
//
//    /////// Wait for the game to start (driver presses PLAY) ///////
//    waitForStart();
//    onPressPlay();
//
//    while (opModeIsActive()) // Run until the end of the match (driver presses STOP)
//    {
//      processController2();
//
//      robot.processCommands();
//
//      createTelemetryMessageForEachLoop();
//    }
//  }
//}
