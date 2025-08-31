//package org.hexnibble.corelib.commands;
//
//import org.hexnibble.corelib.misc.ConfigFile;
//import org.hexnibble.corelib.misc.Field;
//import org.hexnibble.corelib.misc.Pose2D;
//import org.hexnibble.corelib.misc.Timer;
//import org.hexnibble.corelib.misc.Vector2D;
//import org.hexnibble.corelib.motion.pid.dtRotationPIDController;
//import org.hexnibble.corelib.motion.MotionProfile;
//import org.hexnibble.corelib.motion.pid.PIDController;
//import org.hexnibble.corelib.motion.pid.PIDSettings;
//import org.hexnibble.corelib.robot.BaseOdometry;
//
//@Deprecated(since = "5/11/25")
//public class DrivetrainRC extends RobotCommand {
//  protected Pose2D allianceCentricStartingPose;
//  protected Pose2D allianceCentricDeltaPose;
//  protected Pose2D allianceCentricEndingPose;
//
//  public enum COMMAND_TYPE {
//    ABSOLUTE,
//    DELTA
//  }
//
//  protected COMMAND_TYPE commandType;
//
//  public enum COMMAND_CONTENTS {
//    TRANSLATION,
//    ROTATION,
//    TRANSLATION_AND_ROTATION
//  }
//
//  protected COMMAND_CONTENTS commandContents;
//
//  BaseOdometry odometry;
//  protected Pose2D PIDControllerValues = new Pose2D(0.0, 0.0, 0.0);
//
//  protected MotionProfile translationMotionProfile;
//  protected MotionProfile rotationMotionProfile;
//  protected PIDController XPIDController;
//  protected PIDController YPIDController;
//  protected dtRotationPIDController rotationPIDController;
//  protected double v_max;
//  protected double a_max;
//  protected double omega_max;
//  protected double alpha_max;
//
//  protected double distanceToTranslationTarget_mm;
//  protected double distanceToAngleTargetRadians;
//  protected boolean translationTargetCounterActivated = false;
//  protected boolean rotationTargetCounterActivated = false;
//
//  public enum ROTATION_DIRECTION {
//    CLOCKWISE,
//    COUNTERCLOCKWISE
//  }
//
//  protected ROTATION_DIRECTION rotationDirection;
//
//  /**
//   * Full constructor. Targets are alliance-centric CF.
//   *
//   * @param commandID String label for this command
//   * @param commandType Type of command (ABSOLUTE or DELTA)
//   * @param commandContents ROTATION, TRANSLATION, OR ROTATION_AND_TRANSLATION
//   * @param allianceCentricPose Target change in x, y, and IMU-style heading (radians) in
//   *     alliance-centric POV. Counter-clockwise turns are positive.
//   * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
//   * @param odometry Odometry object
//   * @param xyPIDSettings PID settings for translation movements
//   * @param rotationPIDSettings PID settings for rotation movements
//   * @param maxCommandDuration_ms Maximum duration of this command in ms
//   */
//  public DrivetrainRC(
//      String commandID,
//      COMMAND_TYPE commandType,
//      COMMAND_CONTENTS commandContents,
//      Pose2D allianceCentricPose,
//      ROTATION_DIRECTION rotationDirection,
//      BaseOdometry odometry,
//      PIDSettings xyPIDSettings,
//      PIDSettings rotationPIDSettings,
//      int maxCommandDuration_ms) {
//
//    super(commandID, maxCommandDuration_ms);
//
//    this.commandType = commandType;
//    this.commandContents = commandContents;
//
//    if (this.commandType == COMMAND_TYPE.ABSOLUTE) {
//      allianceCentricEndingPose = allianceCentricPose;
//    } else { // Type DELTA
//      allianceCentricDeltaPose = allianceCentricPose;
//    }
//    this.rotationDirection = rotationDirection;
//
//    v_max = ConfigFile.DRIVETRAIN_V_MAX;
//    a_max = ConfigFile.DRIVETRAIN_A_MAX;
//    omega_max = ConfigFile.DRIVETRAIN_OMEGA_MAX;
//    alpha_max = ConfigFile.DRIVETRAIN_ALPHA_MAX;
//
//    this.odometry = odometry;
//
//    // Set up translation and rotation PID Controllers
//    PIDSettings xySettings =
//        new PIDSettings(
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ks,
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kp,
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ki,
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kd);
//    PIDSettings rotationSettings =
//        new PIDSettings(
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Ks,
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Kp,
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Ki,
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Kd);
//
//    // Create PID Controllers
//    double translationTargetTolerance = 5.0;
//    double rotationTargetToleranceDegrees = 2.0;
//
//    XPIDController = new PIDController(xySettings, translationTargetTolerance); // Kp = 0.01
//    YPIDController = new PIDController(xySettings, translationTargetTolerance); // Kp = 0.01
////    rotationPIDController =
////        new dtRotationPIDController(
////            rotationSettings,
////            Math.toRadians(rotationTargetToleranceDegrees),
////            this.rotationDirection); // Kp = 0.01
//  }
//
//  /**
//   * Constructor without PID settings. Targets are alliance-centric CF.
//   *
//   * @param commandID String label for this command
//   * @param commandType Type of command (ABSOLUTE or DELTA)
//   * @param commandContents ROTATION, TRANSLATION, OR ROTATION_AND_TRANSLATION
//   * @param allianceCentricPose Target change in x, y, and IMU-style heading (radians) in
//   *     alliance-centric POV. Counter-clockwise turns are positive.
//   * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
//   * @param odometry Odometry object
//   * @param maxCommandDuration_ms Maximum duration of this command in ms
//   */
//  public DrivetrainRC(
//      String commandID,
//      COMMAND_TYPE commandType,
//      COMMAND_CONTENTS commandContents,
//      Pose2D allianceCentricPose,
//      ROTATION_DIRECTION rotationDirection,
//      BaseOdometry odometry,
//      int maxCommandDuration_ms) {
//
//    this(
//        commandID,
//        commandType,
//        commandContents,
//        allianceCentricPose,
//        rotationDirection,
//        odometry,
//        new PIDSettings(
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ks,
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kp,
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ki,
//            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kd),
//        new PIDSettings(
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Ks,
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Kp,
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Ki,
//            ConfigFile.DRIVETRAIN_ROTATION_PID_Kd),
//        maxCommandDuration_ms);
//  }
//
//  @Override
//  public void initializeCommand() {
//    super.initializeCommand();
//  }
//
//  @Override
//  public void onStart() {
//    super.onStart();
//
//    allianceCentricStartingPose = odometry.getPoseEstimate(); // Store the starting pose
//
//    // For movements to a particular coordinate
//    if (commandType == COMMAND_TYPE.ABSOLUTE) {
//      allianceCentricDeltaPose = new Pose2D();
//      allianceCentricDeltaPose.x = allianceCentricEndingPose.x - allianceCentricStartingPose.x;
//      allianceCentricDeltaPose.y = allianceCentricEndingPose.y - allianceCentricStartingPose.y;
//      allianceCentricDeltaPose.heading =
//          Field.enforceIMUHeadingRangeRadians(
//              allianceCentricEndingPose.heading - allianceCentricStartingPose.heading);
//    }
//
//    // Set up translation motor profile
//    distanceToTranslationTarget_mm =
//        Math.sqrt(
//            (allianceCentricDeltaPose.x * allianceCentricDeltaPose.x)
//                + (allianceCentricDeltaPose.y * allianceCentricDeltaPose.y));
//    translationMotionProfile = new MotionProfile(distanceToTranslationTarget_mm, v_max, a_max);
//
//    // Set up rotation motion profile
//    distanceToAngleTargetRadians = allianceCentricDeltaPose.heading;
//    rotationMotionProfile =
//        new MotionProfile(
//            distanceToAngleTargetRadians, Math.toRadians(omega_max), Math.toRadians(alpha_max));
//  }
//
//  public COMMAND_CONTENTS getCommandType() {
//    return commandContents;
//  }
//
//  public Pose2D getCurrentPIDControllerValues() {
//    return PIDControllerValues;
//  }
//
//  @Override
//  public boolean processCommand() {
//
//    Pose2D currentAllianceCentricPose =
//        odometry.getPoseEstimate(); // Get current pose (alliance-centric)
//    long elapsedTime = getElapsedCommandTime(Timer.TimerUnit.ms); // Get elapsed time
//
//    // Obtain how far along the profile this command should be at this time
//    double linearDistance = translationMotionProfile.getProfiledValue(elapsedTime);
//    double percentOfTotalLinearDistance;
//    if (distanceToTranslationTarget_mm == 0.0) {
//      percentOfTotalLinearDistance = 1.0;
//    } else {
//      percentOfTotalLinearDistance = linearDistance / distanceToTranslationTarget_mm;
//    }
//
//    double angularDistance = rotationMotionProfile.getProfiledValue(elapsedTime);
//    double percentOfTotalAngularDistance;
//    if (distanceToAngleTargetRadians == 0.0) {
//      percentOfTotalAngularDistance = 1.0;
//    } else {
//      percentOfTotalAngularDistance = angularDistance / distanceToAngleTargetRadians;
//    }
//
//    // If target counters in PID controllers have not been activated, and movement is >90% of the
//    // distance
//    // then activate the target counters
//    //        if (!translationTargetCounterActivated && (percentOfTotalLinearDistance > 0.90)) {
//    //            XPIDController.activateTargetCounter();
//    //            YPIDController.activateTargetCounter();
//    //            translationTargetCounterActivated = true;
//    //        }
//    //        if (!rotationTargetCounterActivated && (percentOfTotalAngularDistance > 0.90)) {
//    //            rotationPIDController.activateTargetCounter();
//    //            rotationTargetCounterActivated = true;
//    //        }
//
//    // Calculate the expected current location based on the above percentage of time elapsed on the
//    // motion profile.
//    Vector2D currentMotionProfileTargetCoords =
//        new Vector2D(
//            (allianceCentricDeltaPose.x * percentOfTotalLinearDistance)
//                + allianceCentricStartingPose.x,
//            (allianceCentricDeltaPose.y * percentOfTotalLinearDistance)
//                + allianceCentricStartingPose.y);
//
//    // Calculate the error by comparing the actual current location with the expected location
//    double errorX = currentMotionProfileTargetCoords.x - currentAllianceCentricPose.x;
//    double errorY = currentMotionProfileTargetCoords.y - currentAllianceCentricPose.y;
//
//    double currentMotionProfileTargetHeadingRadians =
//        Field.addRadiansToIMUHeading(
//            allianceCentricStartingPose.heading,
//            allianceCentricDeltaPose.heading * percentOfTotalAngularDistance);
//    double errorHeadingRadians =
//        Field.addRadiansToIMUHeading(
//            currentMotionProfileTargetHeadingRadians, -currentAllianceCentricPose.heading);
//
//    // Update PIDController values
//    PIDControllerValues.x = XPIDController.calculateNewControlValue(errorX);
//    PIDControllerValues.y = YPIDController.calculateNewControlValue(errorY);
//    PIDControllerValues.heading =
//        rotationPIDController.calculateNewControlValue(errorHeadingRadians);
//
//    // Check whether command is complete
//    if (commandContents == COMMAND_CONTENTS.TRANSLATION) {
//      if (XPIDController.isCommandComplete() && YPIDController.isCommandComplete()) {
//        currentCommandPhase = COMMAND_COMPLETE;
//      }
//    } else if (commandContents == COMMAND_CONTENTS.ROTATION) {
//      if (rotationPIDController.isCommandComplete()) {
//        currentCommandPhase = COMMAND_COMPLETE;
//      }
//    } else {
//      if (XPIDController.isCommandComplete()
//          && YPIDController.isCommandComplete()
//          && rotationPIDController.isCommandComplete()) {
//        currentCommandPhase = COMMAND_COMPLETE;
//      }
//    }
//
//    return true;
//  }
//}
