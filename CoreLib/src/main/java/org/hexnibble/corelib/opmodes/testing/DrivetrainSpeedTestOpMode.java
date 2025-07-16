//package org.hexnibble.corelib.opmodes.testing;
//
//import android.util.Log;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.io.PrintWriter;
//import java.util.ArrayList;
//import org.hexnibble.corelib.misc.Pose2D;
//import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
//import org.hexnibble.corelib.robot.CoreRobot;
//import org.hexnibble.corelib.wrappers.controller.ControllerWrapper;
//
//@TeleOp(name = "Drivetrain Speed Test Op Mode", group = "Test OpModes")
//@Disabled
//// Remove or comment out the @Disabled line to display this OpMode in the Driver Station OpMode list
//
//public class DrivetrainSpeedTestOpMode extends CoreLinearOpMode {
//  private Pose2D robotPreviousPose;
//  private Pose2D robotPose;
//
//  private enum TEST_TYPE {
//    FORWARD,
//    LEFT,
//    CW_SPIN
//  }
//
//  private TEST_TYPE activeTestType;
//
//  private PrintWriter printWriter = null;
//
//  public DrivetrainSpeedTestOpMode() {
//    super(OP_MODE_TYPE.TELE_OP);
//  }
//
//  /** Process controller 2 buttons */
//  protected void processController2() {
//    controller2.updateGamepadData();
//
//    // Test buttons
//    if (controller2.isButtonPressed(ControllerWrapper.BUTTON_NAME.dpad_up)) {
//      activeTestType = TEST_TYPE.FORWARD;
//      robot.setDrivetrainManualMovement_Y(1.0);
//    } else if (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.dpad_down)) {
//    } else if (controller2.isButtonPressed(ControllerWrapper.BUTTON_NAME.dpad_left)) {
//      activeTestType = TEST_TYPE.LEFT;
//      robot.setDrivetrainManualMovement_X(-1.0);
//    } else if (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.dpad_right)) {
//    } else if (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.left_bumper)) {
//    } else if (controller2.isButtonPressed(ControllerWrapper.BUTTON_NAME.right_bumper)) {
//      activeTestType = TEST_TYPE.CW_SPIN;
//      robot.setDrivetrainManualMovement_Spin(1.0);
//    } else if (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.cross)) {
//      Log.i(TAG, "Canceling run");
//      robot.cancelCurrentDrivetrainRobotCommand();
//      switch (activeTestType) {
//        case FORWARD -> robot.setDrivetrainManualMovement_Y(0.0);
//        case LEFT -> robot.setDrivetrainManualMovement_X(0.0);
//        case CW_SPIN -> robot.setDrivetrainManualMovement_Spin(0.0);
//        default -> throw new IllegalStateException("Unexpected value: " + activeTestType);
//      }
//      activeTestType = null;
//    }
//
//    if (activeTestType != null) {
//      Pose2D pose = robot.getRobotPoseEstimate();
//      if (pose != null) {
//        printWriter.printf(
//            "%d\t%.2f\t%.2f\t%.2f\t\n",
//            robot.getLastPoseEstimateTime(), pose.x, pose.y, Math.toDegrees(pose.heading));
//
//        //                printWriter.printf("%.2f\t%.2f\t%.2f\t%.2f\t\n", getRuntime(TimerUnit.ms),
//        // pose.x, pose.y, Math.toDegrees(pose.heading));
//        //                Log.i(CoreRobot.TAG, pose.x + "\t" + pose.y + "\t" +
//        // Math.toDegrees(pose.heading));
//      }
//    }
//  }
//
//  @Override
//  protected void addTelemetryBody() {
//    telemetry.addLine("Drivetrain Test OpMode INSTRUCTIONS (Use Controller 2):");
//    telemetry.addLine("\tDPAD UP for forward test.");
//    telemetry.addLine("\tDPAD LEFT for left strafing test.");
//    telemetry.addLine("\tRight bumper for CW spin test.");
//    telemetry.addLine("\tX to cancel test.");
//    telemetry.addLine("");
//
//    // Add encoder info
//    ArrayList<Double> encoderPositionList = robot.getOdometryEncoderPositions_mm();
//
//    if (encoderPositionList != null) {
//      for (int i = 0; i < encoderPositionList.size(); i++) {
//        telemetry.addData("Encoder Position (mm): ", "%.2f", encoderPositionList.get(i));
//      }
//      telemetry.addLine("\n");
//
//      // Add odometry pose info
//      Pose2D pose = robot.getRobotPoseEstimate();
//      if (pose != null) {
//        telemetry.addData("Robot Pose: x (mm)=", "%.2f", pose.x);
//        telemetry.addData("Robot Pose: y (mm)=", "%.2f", pose.y);
//        telemetry.addData("Robot Pose: hdg (deg)=", "%.2f", Math.toDegrees(pose.heading));
//      }
//    }
//  }
//
//  @Override
//  public void runOpMode() {
//    initializeOpMode();
//    robotPreviousPose = new Pose2D(robot.getRobotPoseEstimate());
//    robotPose = new Pose2D(robotPreviousPose);
//
//    try {
//      FileWriter fileWriter = new FileWriter("/sdcard/FIRST/DrivetrainTestData.txt");
//      printWriter = new PrintWriter(fileWriter);
//      printWriter.print(
//          "Update Time(ms)\tAlliance CF Pose x (mm)\tAlliance CF Pose y (mm)\tAlliance CF Hdg (deg)\n");
//    } catch (IOException e) {
//      String errorMessage = "Error opening /sdcard/FIRST/DrivetrainTestData.txt: " + e.getMessage();
//      Log.i(CoreRobot.TAG, errorMessage);
//      System.out.println(errorMessage);
//      e.printStackTrace();
//    }
//
//    telemetry.addLine("READY.");
//    telemetry.update();
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
//
//    if (printWriter != null) {
//      printWriter.close();
//    }
//  }
//}
