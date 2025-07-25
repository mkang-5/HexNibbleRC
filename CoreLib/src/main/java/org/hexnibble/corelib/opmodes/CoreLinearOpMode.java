package org.hexnibble.corelib.opmodes;

import static org.hexnibble.corelib.wrappers.controller.ButtonWrapper.BUTTON_STATE.*;
import static org.hexnibble.corelib.wrappers.controller.ControllerWrapper.ANALOG_STICK.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.hexnibble.corelib.commands.DrivetrainRC;
import org.hexnibble.corelib.commands.rc.RCController;
import org.hexnibble.corelib.exception.StopOpModeException;
import org.hexnibble.corelib.misc.AllianceInfo;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Timer;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.Waypoint;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.robot.MecanumDrivetrain;
import org.hexnibble.corelib.wrappers.controller.AnalogStickToFunction;
import org.hexnibble.corelib.wrappers.controller.ButtonToFunction;
import org.hexnibble.corelib.wrappers.controller.ControllerWrapper;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/** This is an abstract base class for Autonomous and Driver-controlled OpModes. */
public abstract class CoreLinearOpMode extends LinearOpMode {
  public enum OP_MODE_TYPE { AUTO, TELE_OP }
  protected OP_MODE_TYPE opModeType;

  protected RCController rcController;

  protected static boolean didAUTOOpModeJustCompleteSuccessfully;
  protected static HashMap<String, Double> servoPositionMap = new HashMap<>();

  // Configuration variables
  protected static final String configFileName = "Robot_Config.txt";
  protected ConfigFile configFile;

  // Controller Variables
  protected ControllerWrapper controller1, controller2;

  // Robot Variables
  protected static CoreRobot robot;
  protected MecanumDrivetrain d;
  protected static HardwareMap currentHardwareMap;

  // Pathing Variables
  protected Pose2D startingFieldPose;

  // PedroPathing
//  protected Follower pedroFollower;
//  protected Pose startingPedroPose;

  // Loop Monitoring
  protected long loopCounter = 0L;
  protected long minLoopTime_ms = 1000L;
  protected long maxLoopTime_ms = 0L;
  protected float averageLoopTime_ms = 0.0f;
  private long currentLoopTime_ms = 0L;
  protected long prevElapsedTime_ms = 0L;
  public static Timer OpModeRunTimer = new Timer();

  protected String className = this.getClass().getSimpleName();

  public CoreLinearOpMode(OP_MODE_TYPE opModeType) {
    Msg.log("\n\n" + className, "CoreLinearOpMode", "***** Starting new OpMode *****");
    this.opModeType = opModeType;

    if (this.opModeType == OP_MODE_TYPE.AUTO) {
      Msg.log(className, "Constructor", "AUTO OpMode so clearing robot");
      robot = null;
    }

    Msg.log(className, "Constructor", "Reading config file " + configFileName);
    configFile = new ConfigFile(configFileName);
    configFile.readFromFile();
  }

  public CoreLinearOpMode(OP_MODE_TYPE opModeType, ConfigFile configFile, Constants constants) {
    Msg.log("\n\n" + className, "", "***** Starting new OpMode *****");
    this.opModeType = opModeType;

    if (this.opModeType == OP_MODE_TYPE.AUTO) {
      Msg.log(className, "Constructor", "AUTO OpMode so clearing robot");
      robot = null;
    }

    Msg.log(className, "Constructor", "Reading config file " + configFileName);
    this.configFile = configFile;
    this.configFile.readFromFile();
  }

  // region ** Main Functions **
  @Override
  public void runOpMode() {
    initializeOpMode();

    rcController = new RCController(robot, opModeType, controller1, controller2);

    telemetry.addLine("Ready.");
    telemetry.update();

    waitForStart();

    // Play pressed
    if (opModeIsActive()) {
      // Stuff to run before OpMode Loop
      onPressPlay();

      while (opModeIsActive()) {

        if (processLoopTime()) {
          rcController.processCommands();

          if (!isStopRequested()) {
            createTelemetryMessageForEachLoop();
          }
//        processCommands();
        }
      }
    }
    onStopOpMode();
  }

  /**
   * This is called at the very start of runOpMode() and contains all the necessary steps to
   * initialize the OpMode: - If AUTO or info not set: - Obtain alliance color and side - Prompt for
   * other initialization info - set starting field pose - create robot object - initialize IMU -
   * create controller
   */
  protected void initializeOpMode() {
    Msg.log(className, "initializeOpMode", "Initializing OpMode Type: " + opModeType);

    // Clear Alliance info if starting an AUTO OpMode
    if (opModeType == OP_MODE_TYPE.AUTO) {
      AllianceInfo.clearAllianceInfo();
      didAUTOOpModeJustCompleteSuccessfully = false; // reset flag
    }

    // Create controllers
//    controller1 = new ControllerWrapper(gamepad1);
//    controller2 = new ControllerWrapper(gamepad2);

    if (!AllianceInfo.isInfoSet()) {
      promptForAndSetAllianceColorAndSide();
    }

    if (opModeType == OP_MODE_TYPE.AUTO) {
      promptForOtherAutoInitInfo();
      setStartingFieldPose(AllianceInfo.getAllianceColor(), AllianceInfo.getAllianceSide());
    }

    //  If the restart robot command was sent from the driver station, the hardware map can change.
    //  So check whether the hardware map has changed, and if so store the new map
    //  and set the robot object to null.
    if (currentHardwareMap != hardwareMap) {
      Msg.log(
              className,
              "refreshCurrentHardwareMap",
              "Hardware map has changed. Deleting any existing robot object and storing new hardwareMap = "
                      + hardwareMap);
      robot = null;
      currentHardwareMap = hardwareMap;
    }

    // If the robot object does not exist (which should also be the case for all AUTO OpModes),
    // create a new robot object. Otherwise, just reset the existing one.
    // Robot must be created AFTER the hardware map check.
    if (robot == null) {
      robot = createRobotObject(currentHardwareMap);
    }
    else {
      robot.resetSystem();
    }
    d = robot.drivetrain;

    if (opModeType == OP_MODE_TYPE.AUTO) {
      robot.setPoseEstimate(Pose2D.convertFieldCFAbsoluteToAllianceCFAbsolute(startingFieldPose,
          AllianceInfo.getAllianceColor()));
    }
    robot.bulkReadControlHub();
    robot.bulkReadExpansionHub();

    // Check battery voltage
    double startingBatteryVoltage = robot.getRobotBatteryVoltage();
    Msg.log(className, "initializeOpMode", "Starting battery voltage: " + startingBatteryVoltage);
    if (startingBatteryVoltage < ConfigFile.ROBOT_MIN_BATTERY_VOLTAGE_FOR_WARNING) {
      Msg.log(className, "initializeOpMode", "Low starting battery voltage!");
    }

    Msg.log(className, "initializeOpMode", "Drivetrain object=" + d);
  }

  protected void onPressPlay() {
    OpModeRunTimer.restartTimer();
    initializeLoopTimer();
    Msg.log(getClass().getSimpleName(), "onStartOpMode", "OpMode Started");
  }

//  protected void processCommands() {
//    rcController.processCommands();
//
//    createTelemetryMessageForEachLoop();
//  }

  protected void onStopOpMode() {
    rcController = null;

    if (opModeType == OP_MODE_TYPE.TELE_OP) {
      didAUTOOpModeJustCompleteSuccessfully = false; // reset flag
    }

    if (robot != null) {
      robot.destructor();
      Msg.logIfDebug(className, "onStopOpMode",
          "Control Hub Bulk Cache Mode=" + robot.getHubBulkCachingMode(CoreRobot.HUB_TYPE.CONTROL_HUB)
              + ", Expansion Hub Bulk Cache Mode=" + robot.getHubBulkCachingMode(CoreRobot.HUB_TYPE.EXPANSION_HUB));
    }
    Msg.log(className, "onStopOpMode", "Avg Loop Time (ms)=" + averageLoopTime_ms
            + ", Min=" + minLoopTime_ms
            + ", Max=" + maxLoopTime_ms);
  }
  // endregion ** Main Functions **

  // region ** Utility Functions **

  /** Create controllers for TeleOp. This function should be overridden for each season's game.
   * But don't forget to call this function first in any child version. */
  protected void createControllersForTeleOp() {
    // Create controllers
    Msg.logIfDebug(className, "createControllersForTeleOp", "Creating controller objects.");

    // region ** Controller 1 **
    controller1 = new ControllerWrapper(gamepad1);
    // Drivetrain movements
    controller1.addActiveStickGroup(
        new AnalogStickToFunction(left_stickX, (x) -> d.setDrivetrainManualMovement_X(x)));
    controller1.addActiveStickGroup(
        new AnalogStickToFunction(left_stickY, (y) -> d.setDrivetrainManualMovement_Y(-y)));
    controller1.addActiveStickGroup(
        new AnalogStickToFunction(ControllerWrapper.ANALOG_STICK.left_trigger,
                (spin) -> d.setDrivetrainManualMovement_Spin(spin)),
        new AnalogStickToFunction(ControllerWrapper.ANALOG_STICK.right_trigger,
                (spin) -> d.setDrivetrainManualMovement_Spin(-spin)));

    // Reset IMU heading
    controller1.addActiveButtonGroup(
        new ButtonToFunction(ControllerWrapper.BUTTON_NAME.square, NEWLY_PRESSED, ControllerWrapper.OPTION_SHIFT,
            () -> robot.resetIMUHeading()));
    // endregion ** Controller 1 **
    // region ** Controller 2 **
    controller2 = new ControllerWrapper(gamepad2);
    // endregion ** Controller 2 **
  }

  /**
   * Create the robot object to use with the OpMode. This is called by initializeOpMode() after
   * prompting for alliance color/side, other init info, and setting the starting field pose. It is
   * only called if the robot object does not exist or an AUTO OpMode is starting
   */
  protected CoreRobot createRobotObject(HardwareMap hwMap) {
    return new CoreRobot(hwMap, "CoreRobot");
  }

  /** Prompt for and set alliance color and side */
  private void promptForAndSetAllianceColorAndSide() {
    try {
      ControllerWrapper controller1 = new ControllerWrapper(gamepad1);
      ControllerWrapper controller2 = new ControllerWrapper(gamepad2);

      AllianceInfo.ALLIANCE_COLOR allianceColor = null;
      while (!isStopRequested() && (allianceColor == null)) {
        telemetry.addLine("Please choose ALLIANCE color:");
        telemetry.addLine("\tLEFT BUMPER = BLUE Alliance");
        telemetry.addLine("\tRIGHT BUMPER = RED Alliance\n");
        telemetry.update();

        controller1.updateGamepadData();
        controller2.updateGamepadData();
        if ((controller1.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.left_bumper))
            || (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.left_bumper))) {
          allianceColor = AllianceInfo.ALLIANCE_COLOR.BLUE;
        } else if ((controller1.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.right_bumper))
            || (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.right_bumper))) {
          allianceColor = AllianceInfo.ALLIANCE_COLOR.RED;
        }
      }
      if (isStopRequested()) throw new StopOpModeException("promptForAllianceColor");

      AllianceInfo.ALLIANCE_SIDE allianceSide = null;
      while (!isStopRequested() && (allianceSide == null)) {
        telemetry.addData("Alliance Color: ", allianceColor);
        telemetry.addLine("\nNow choose starting SIDE:");
        telemetry.addLine("\tLEFT BUMPER = LEFT side");
        telemetry.addLine("\tRIGHT BUMPER = RIGHT Side");
        telemetry.addLine("");
        telemetry.update();

        controller1.updateGamepadData();
        controller2.updateGamepadData();
        if ((controller1.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.left_bumper))
            || (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.left_bumper))) {
          allianceSide = AllianceInfo.ALLIANCE_SIDE.LEFT;
        } else if ((controller1.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.right_bumper))
            || (controller2.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.right_bumper))) {
          allianceSide = AllianceInfo.ALLIANCE_SIDE.RIGHT;
        }
      }
      if (isStopRequested()) throw new StopOpModeException("promptForAllianceSide");

      AllianceInfo.setAllianceInfo(allianceColor, allianceSide);

      Msg.log(
          className,
          "promptForAllianceColorAndSide",
          "Alliance Color=" + allianceColor + ", Alliance Side=" + allianceSide);

      telemetry.addLine("");
      telemetry.update();
    }
    catch (StopOpModeException e) {
      Msg.log(
          className,
          "promptForAndSetAllianceColorAndSide",
          "StopOpMode pressed during " + e.getMessage());
      onStopOpMode();
    }
  }

  /**
   * Override this function to prompt for other info needed during initialization For example,
   * alternate starting positions
   */
  protected void promptForOtherAutoInitInfo() {}

  /**
   * This method sets the starting X,Y coordinates (in mm) of the center of the robot, as well as
   * the heading, in field CF.
   */
  protected void setStartingFieldPose(
      AllianceInfo.ALLIANCE_COLOR allianceColor, AllianceInfo.ALLIANCE_SIDE allianceSide) {
    startingFieldPose = new Pose2D();

    if (allianceColor == AllianceInfo.ALLIANCE_COLOR.BLUE) {
      if (allianceSide == AllianceInfo.ALLIANCE_SIDE.LEFT) {
        startingFieldPose.x = Constants.FIELD_START_POSITION_BLUE_LEFT_X_MM;
        startingFieldPose.y = Constants.FIELD_START_POSITION_BLUE_LEFT_Y_MM;
        startingFieldPose.heading =
            Math.toRadians(Constants.FIELD_START_POSITION_BLUE_LEFT_HDG_DEGREES);
      } else {
        startingFieldPose.x = Constants.FIELD_START_POSITION_BLUE_RIGHT_X_MM;
        startingFieldPose.y = Constants.FIELD_START_POSITION_BLUE_RIGHT_Y_MM;
        startingFieldPose.heading =
            Math.toRadians(Constants.FIELD_START_POSITION_BLUE_RIGHT_HDG_DEGREES);
      }
    } else { // RED Alliance
      if (allianceSide == AllianceInfo.ALLIANCE_SIDE.LEFT) {
        startingFieldPose.x = Constants.FIELD_START_POSITION_RED_LEFT_X_MM;
        startingFieldPose.y = Constants.FIELD_START_POSITION_RED_LEFT_Y_MM;
        startingFieldPose.heading =
            Math.toRadians(Constants.FIELD_START_POSITION_RED_LEFT_HDG_DEGREES);
      } else {
        startingFieldPose.x = Constants.FIELD_START_POSITION_RED_RIGHT_X_MM;
        startingFieldPose.y = Constants.FIELD_START_POSITION_RED_RIGHT_Y_MM;
        startingFieldPose.heading =
            Math.toRadians(Constants.FIELD_START_POSITION_RED_RIGHT_HDG_DEGREES);
      }
    }

    Msg.log(
        className,
        "setStartingFieldPose",
        "Setting starting position (field CF) as: "
            + startingFieldPose.x
            + ", "
            + startingFieldPose.y
            + ", hdg="
            + Math.toDegrees(startingFieldPose.heading));
  }
  // endregion ** Utility Functions **

  // region ** Queue robot command functions **
  /**
   * Queue a robot drivetrain movement to a field location.
   *
   * @param commandID String identifier for this command
   * @param fieldPose X,Y coordinates (Field CF) and heading (IMU-style, radians)
   * @param rotationDirection Direction of rotation
   * @param maxVelocityAdjustment Maximum velocity/acceleration adjustment factor (0 - 1.0)
   * @param maxCommandDuration maximum duration in ms
   */
  @Deprecated
  protected void qRobotMoveTo(
      String commandID,
      Pose2D fieldPose,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      double maxVelocityAdjustment,
      int maxCommandDuration) {
    List<Waypoint> waypointList = new ArrayList<>();
    waypointList.add(new Waypoint(fieldPose));
    qRobotMoveTo(
        commandID, waypointList, rotationDirection, maxVelocityAdjustment, maxCommandDuration);
    //        robot.qAbsoluteDrivetrainRC(commandID, waypointList, rotationDirection);
  }

  /**
   * Queue a robot drivetrain movement to a list of field waypoint locations.
   *
   * @param commandID String identifier for this command
   * @param waypointList List of waypoints in Field CF--X,Y coordinates and heading (IMU-style,
   *     radians)
   * @param rotationDirection Direction of rotation
   * @param maxVelocityAdjustment Maximum velocity/acceleration adjustment factor (0 - 1.0)
   * @param maxCommandDuration maximum duration in ms
   */
  @Deprecated
  protected void qRobotMoveTo(
      String commandID,
      List<Waypoint> waypointList,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      double maxVelocityAdjustment,
      int maxCommandDuration) {
    // Convert each waypoint from field CF to alliance CF
    waypointList.forEach(
        waypoint -> {
          Pose2D targetPose = waypoint.getTargetPose();
          targetPose =
              Pose2D.convertFieldCFAbsoluteToAllianceCFAbsolute(
                  targetPose, AllianceInfo.getAllianceColor());
          waypoint.setTargetPose(targetPose);
        });

    robot.qAbsoluteDrivetrainRC(
        commandID, waypointList, rotationDirection, maxVelocityAdjustment, maxCommandDuration);
  }

  /**
   * This is only for PID testing OpMode
   *
   * @param commandID
   * @param allianceCFDeltaPose
   * @param rotationDirection
   * @param xyPIDSettings
   * @param rotationPIDSettings
   */
  @Deprecated
  protected void qDeltaDrivetrainRC(
      String commandID,
      Pose2D allianceCFDeltaPose,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      PIDSettings xyPIDSettings,
      PIDSettings rotationPIDSettings) {
    robot.qDeltaDrivetrainRC(
        commandID, allianceCFDeltaPose, rotationDirection, xyPIDSettings, rotationPIDSettings);
  }

  /*
      protected void qDrivetrainSpinTurnToNearest45(String commandID,
                                                    DrivetrainRC.ROTATION_DIRECTION rotationDirection,
                                                    PIDSettings xyPIDSettings, PIDSettings rotationPIDSettings, int maxCommandDuration) {
          robot.qDrivetrainSpinTurnToNearest45(commandID, rotationDirection, xyPIDSettings, rotationPIDSettings, maxCommandDuration);
      }
  */
  // endregion ** Queue robot command functions **

  // region ** Loop Timer Functions **
  /**
   * Call this function immediately prior to the first OpMode loop to initialize variables needed
   * for the loop timer.
   */
  protected void initializeLoopTimer() {
    loopCounter = 1;
    prevElapsedTime_ms = OpModeRunTimer.getElapsedTime(Timer.TimerUnit.ms);
  }

  /**
   * Calculate the time taken for the current loop (since the last time this function was called).
   * Call this function towards the end of each loop.
   *
   * @return True/False whether the loop time has met the threshold minimum loop time
   */
  private boolean processLoopTime() {
    final long currentElapsedTime_ms = OpModeRunTimer.getElapsedTime(Timer.TimerUnit.ms);

    currentLoopTime_ms = (currentElapsedTime_ms - prevElapsedTime_ms);

    if (currentLoopTime_ms > Constants.MINIMUM_OP_MODE_LOOP_TIME_MS) {
      if (currentLoopTime_ms > Constants.LOOP_TIME_THRESHOLD_FOR_LOGGING_MS) {
        Msg.log(
          className, "addTelemetryLoopTimes", "Loop time = "
            + currentLoopTime_ms + " ms, which exceeds logging threshold ("
            + Constants.LOOP_TIME_THRESHOLD_FOR_LOGGING_MS + " ms)");
      }

      if (currentLoopTime_ms > maxLoopTime_ms) {
        maxLoopTime_ms = currentLoopTime_ms;
      }
      else if (currentLoopTime_ms < minLoopTime_ms) {
        minLoopTime_ms = currentLoopTime_ms;
      }

      averageLoopTime_ms = (float) currentElapsedTime_ms / loopCounter;

      loopCounter += 1;
      prevElapsedTime_ms = currentElapsedTime_ms;
      return true;
    }
    else return false;
  }

  // endregion ** Loop Timer Functions **
  // region ** Telemetry Functions **
  protected void createTelemetryMessageForEachLoop() {
    addTelemetryHeader();
    addTelemetryBody();
    addLoopTimeInfoToTelemetry(currentLoopTime_ms);
    telemetry.update();
  }

  /** Add initial lines to telemetry. */
  protected void addTelemetryHeader() {
    telemetry.addLine();
  }

  /** Create the body of the telemetry info. Override this function to include relevant info. */
  protected void addTelemetryBody() {
    telemetry.addData("\nCurrent IMU Heading (deg): ", "%.2f", robot.getStoredIMUHeadingDegrees());

    // Add encoder info
    ArrayList<Double> encoderPositionList = robot.getOdometryEncoderPositions_mm();
    if (encoderPositionList != null) {
      for (int i = 0; i < encoderPositionList.size(); i++) {
        telemetry.addData("Encoder Position (mm): ", "%.2f", encoderPositionList.get(i));
      }
      telemetry.addLine("\n");

      // Add odometry pose info
      Pose2D pose = robot.getRobotPoseEstimate();
      if (pose != null) {
        Pose2D fieldCFPose =
            Pose2D.convertAllianceCFAbsoluteToFieldCFAbsolute(
                pose, AllianceInfo.getAllianceColor());

        telemetry.addData("Robot Pose: Field CF x (mm)=", "%.2f", fieldCFPose.x);
        telemetry.addData("Robot Pose: Field CF y (mm)=", "%.2f", fieldCFPose.y);
        telemetry.addData(
            "Robot Pose: Field CF hdg (deg)=", "%.2f", Math.toDegrees(fieldCFPose.heading));

        telemetry.addData("Robot Pose: Alliance CF x (mm)=", "%.2f", pose.x);
        telemetry.addData("Robot Pose: Alliance CF y (mm)=", "%.2f", pose.y);
        telemetry.addData(
            "Robot Pose: Alliance CF hdg (deg)=", "%.2f", Math.toDegrees(pose.heading));
      }
    }
  }

  /** Report loop times in telemetry. */
  private void addLoopTimeInfoToTelemetry(final long currentLoopTime_ms) {
    telemetry.addData("\nAvg Time per Loop (ms):", "%.4f", averageLoopTime_ms);
    telemetry.addLine("Last loop time (ms): " + currentLoopTime_ms);
    telemetry.addLine("Min/Max Loop Time (ms): " + minLoopTime_ms + " / " + maxLoopTime_ms);
//    telemetry.addData("Max Loop Time (ms): ", +maxLoopTime_ms);
    telemetry.addLine();
  }
  // endregion ** Telemetry Functions **
}
