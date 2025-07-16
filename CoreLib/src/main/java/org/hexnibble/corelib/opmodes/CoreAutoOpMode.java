//package org.hexnibble.corelib.opmodes;
//
//import android.util.Log;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import java.util.List;
//import org.hexnibble.corelib.commands.DrivetrainRC;
//import org.hexnibble.corelib.commands.rc.RCController;
//import org.hexnibble.corelib.misc.AllianceInfo;
//import org.hexnibble.corelib.misc.ConfigFile;
//import org.hexnibble.corelib.misc.Constants;
//import org.hexnibble.corelib.misc.Field;
//import org.hexnibble.corelib.misc.Pose2D;
//import org.hexnibble.corelib.motion.Waypoint;
//
//@Autonomous(name = "Basic Linear Autonomous with Mecanum Drivetrain", group = "Linear OpMode")
//@Disabled
//public class CoreAutoOpMode extends CoreLinearOpMode {
//  protected CoreOpModeProgram autoProgram;
//
//  // Constants storing the various possible states of the robot during autonomous
////  protected final int STATE_INITIAL = 0b000000000000000;
////  protected final int STATE_AUTO_COMPLETE = 0b111111111111111;
////  protected int currentPhase = STATE_INITIAL;
//
////  protected boolean hasCurrentPhaseStarted;
//
//  public CoreAutoOpMode(ConfigFile configFile, Constants constants) {
//    super(OP_MODE_TYPE.AUTO, configFile, constants);
//  }
//
//  /**
//   * Display reminders on the driver station screen about appropriate setup (preloading of items).
//   * This function should be overridden for each season.
//   */
//  protected void performAutonomousChecks() {}
//
//  protected void createAutoProgram() {
//    autoProgram = new CoreOpModeProgram(rcController, pedroFollower, startingPedroPose);
//  }
//
//  /**
//   * Queue movement to a location. Coordinates are field-centric and should be for red alliance
//   * movements. They will automatically be mirrored to blue.
//   *
//   * @param commandID String name for this command
//   * @param x X Coordinate (field CF)
//   * @param y Y Coordinate (field CF)
//   * @param hdgDegrees Heading (degrees; IMU-style field CF)
//   * @param rotDir CLOCKWISE or COUNTERCLOCKWISE
//   * @param maxVelocityAdjustment Maximum velocity/acceleration adjustment factor (0 - 1.0)
//   * @param maxCommandDuration maximum duration in ms
//   */
//  @Deprecated(since = "5/16/25", forRemoval = true)
//  protected void qMoveToLocation(
//      String commandID,
//      double x,
//      double y,
//      double hdgDegrees,
//      DrivetrainRC.ROTATION_DIRECTION rotDir,
//      double maxVelocityAdjustment,
//      int maxCommandDuration) {
//    Pose2D fieldPose = new Pose2D(x, y, Math.toRadians(hdgDegrees));
//
//    if (AllianceInfo.getAllianceColor() == AllianceInfo.ALLIANCE_COLOR.BLUE) {
//      fieldPose = Field.mirrorDiagonalRedToBlueAlliancePoses(fieldPose);
//    }
//
//    qRobotMoveTo(commandID, fieldPose, rotDir, maxVelocityAdjustment, maxCommandDuration);
//  }
//
//  @Deprecated(since = "5/16/25", forRemoval = true)
//  protected void qMoveToLocation(
//      String commandID,
//      List<Waypoint> waypointList,
//      DrivetrainRC.ROTATION_DIRECTION rotDir,
//      double maxVelocityAdjustment,
//      int maxCommandDuration) {
//    if (AllianceInfo.getAllianceColor() == AllianceInfo.ALLIANCE_COLOR.BLUE) {
//      Log.i(TAG, "Offsetting coordinates for blue.");
//      waypointList.forEach(
//          waypoint -> {
//            Pose2D bluePose = Field.mirrorDiagonalRedToBlueAlliancePoses(waypoint.getTargetPose());
//            waypoint.setTargetPose(bluePose);
//          });
//
//      if (rotDir == DrivetrainRC.ROTATION_DIRECTION.CLOCKWISE) {
//        rotDir = DrivetrainRC.ROTATION_DIRECTION.COUNTERCLOCKWISE;
//      } else {
//        rotDir = DrivetrainRC.ROTATION_DIRECTION.CLOCKWISE;
//      }
//    }
//
//    qRobotMoveTo(commandID, waypointList, rotDir, maxVelocityAdjustment, maxCommandDuration);
//  }
//
//  @Override
//  public void runOpMode() {
//    initializeOpMode();
//
//    performAutonomousChecks();
//    rcController = new RCController(robot, pedroFollower, opModeType, controller1, controller2);
//    createAutoProgram();
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
//
//      while (opModeIsActive()
//          && !autoProgram.getProgramComplete()) // Run until the end of AUTO or until STOP is pressed
//      {
//        rcController.processCommands();
//
//        processLoopTime();
//        createTelemetryMessageForEachLoop();
//      }
//    } else {
//      Log.i(TAG, "OpMode not active. STOP must have been pressed before OpMode began.");
//    }
//
//    onStopOpMode();
//  }
//}
