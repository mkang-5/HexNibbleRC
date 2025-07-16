package org.hexnibble.corelib.commands;

import android.util.Log;
import java.util.List;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Timer;
import org.hexnibble.corelib.misc.Vector2D;
import org.hexnibble.corelib.motion.pid.dtRotationPIDController;
import org.hexnibble.corelib.motion.MotionProfile;
import org.hexnibble.corelib.motion.pid.PIDController;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.Waypoint;
import org.hexnibble.corelib.robot.BaseOdometry;

@Deprecated(since = "5/11/25")
public class MultiDrivetrainRC extends RobotCommand {
  protected Pose2D allianceCFStartingPose;
  protected Pose2D allianceCFDeltaPose;
  //    protected Pose2D allianceCFTargetPose;
  //    protected Pose2D allianceCFSegmentStartingPose;

  protected List<Waypoint> waypointList;
  protected int currentSegmentBeingTraveled = 0;
  protected boolean isSegmentComplete = false;
  protected double distanceOfCompletedSegments_mm = 0;

  public enum COMMAND_TYPE {
    ABSOLUTE,
    DELTA
  }

  protected COMMAND_TYPE commandType;

  public enum COMMAND_CONTENTS {
    TRANSLATION,
    ROTATION,
    TRANSLATION_AND_ROTATION
  }

  protected COMMAND_CONTENTS commandContents;

  BaseOdometry odometry;
  protected Pose2D PIDControllerValues = new Pose2D(0.0, 0.0, 0.0);

  protected MotionProfile translationMotionProfile;
  protected MotionProfile rotationMotionProfile;
  protected PIDController XPIDController;
  protected PIDController YPIDController;
  protected dtRotationPIDController rotationPIDController;
  protected double v_max;
  protected double a_max;
  protected double omega_max;
  protected double alpha_max;
  protected double maxVelocityAdjustment;

  protected double distanceToTranslationTarget_mm;
  protected double distanceToAngleTargetRadians;
  protected boolean translationTargetCounterActivated = false;
  protected boolean rotationTargetCounterActivated = false;

  //    public enum ROTATION_DIRECTION { CLOCKWISE, COUNTERCLOCKWISE }
  protected DrivetrainRC.ROTATION_DIRECTION rotationDirection;

  protected final double TRANSLATION_TARGET_TOLERANCE_WAYPOINT = 20.0;
  protected final double TRANSLATION_TARGET_TOLERANCE_FINAL = 1.0;
  protected final double ROTATION_TARGET_TOLERANCE_WAYPOINT = 10.0;
  protected final double ROTATION_TARGET_TOLERANCE_FINAL = 1.0;

  /**
   * Constructor without PID Settings. Targets are alliance-centric CF.
   *
   * @param commandID String label for this command
   * @param commandType Type of command (ABSOLUTE or DELTA)
   * @param commandContents ROTATION, TRANSLATION, OR ROTATION_AND_TRANSLATION
   * @param waypointList List of waypoints
   * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
   * @param odometry Odometry object
   * @param xyPIDSettings PID settings for translation movements
   * @param rotationPIDSettings PID settings for rotation movements
   * @param maxCommandDuration_ms Maximum duration of this command in ms
   */
  public MultiDrivetrainRC(
      String commandID,
      COMMAND_TYPE commandType,
      COMMAND_CONTENTS commandContents,
      List<Waypoint> waypointList,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      BaseOdometry odometry,
      PIDSettings xyPIDSettings,
      PIDSettings rotationPIDSettings,
      double maxVelocityAdjustment,
      int maxCommandDuration_ms) {
    super(commandID, maxCommandDuration_ms);

    this.commandType = commandType;
    this.commandContents = commandContents;
    this.waypointList = waypointList;
    /*
            if (this.commandType == COMMAND_TYPE.ABSOLUTE) {
                this.allianceCentricEndingPose = allianceCentricPose;
            }
            else {      // Type DELTA
                this.allianceCentricDeltaPose = allianceCentricPose;
            }
    */
    this.rotationDirection = rotationDirection;

    this.maxVelocityAdjustment = maxVelocityAdjustment;
    this.v_max = ConfigFile.DRIVETRAIN_V_MAX * maxVelocityAdjustment;
    this.a_max = ConfigFile.DRIVETRAIN_A_MAX * maxVelocityAdjustment;
    this.omega_max = ConfigFile.DRIVETRAIN_OMEGA_MAX * maxVelocityAdjustment;
    this.alpha_max = ConfigFile.DRIVETRAIN_ALPHA_MAX * maxVelocityAdjustment;

    this.odometry = odometry;

    // Create PID Controllers
    double translationTargetTolerance;
    double rotationTargetToleranceDegrees;
    if (this.waypointList.size() == 1) {
      translationTargetTolerance = TRANSLATION_TARGET_TOLERANCE_FINAL;
      rotationTargetToleranceDegrees = ROTATION_TARGET_TOLERANCE_FINAL;
    } else {
      translationTargetTolerance = TRANSLATION_TARGET_TOLERANCE_WAYPOINT;
      rotationTargetToleranceDegrees = ROTATION_TARGET_TOLERANCE_WAYPOINT;
    }

    this.XPIDController = new PIDController(xyPIDSettings, translationTargetTolerance); // Kp = 0.01
    this.YPIDController = new PIDController(xyPIDSettings, translationTargetTolerance); // Kp = 0.01
//    this.rotationPIDController =
//        new dtRotationPIDController(
//            rotationPIDSettings,
//            Math.toRadians(rotationTargetToleranceDegrees),
//            this.rotationDirection); // Kp = 0.01
  }

  /**
   * Constructor without PID Settings. Targets are alliance-centric CF.
   *
   * @param commandID String label for this command
   * @param commandType Type of command (ABSOLUTE or DELTA)
   * @param commandContents ROTATION, TRANSLATION, OR ROTATION_AND_TRANSLATION
   * @param waypointList List of waypoints
   * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
   * @param odometry Odometry object
   * @param maxVelocityAdjustment Adjustment factor for maximum velocities/accelerations
   * @param maxCommandDuration_ms Maximum duration of this command in ms
   */
  public MultiDrivetrainRC(
      String commandID,
      COMMAND_TYPE commandType,
      COMMAND_CONTENTS commandContents,
      List<Waypoint> waypointList,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      BaseOdometry odometry,
      double maxVelocityAdjustment,
      int maxCommandDuration_ms) {

    this(
        commandID,
        commandType,
        commandContents,
        waypointList,
        rotationDirection,
        odometry,
        new PIDSettings(
            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ks,
            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kp,
            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ki,
            ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kd),
        new PIDSettings(
            ConfigFile.DRIVETRAIN_ROTATION_PID_Ks,
            ConfigFile.DRIVETRAIN_ROTATION_PID_Kp,
            ConfigFile.DRIVETRAIN_ROTATION_PID_Ki,
            ConfigFile.DRIVETRAIN_ROTATION_PID_Kd),
        maxVelocityAdjustment,
        maxCommandDuration_ms);
  }

  @Override
  public void initializeCommand() {
    super.initializeCommand();
  }

  @Override
  public void onStart() {
    super.onStart();

    Waypoint firstWaypoint = waypointList.get(0);
    firstWaypoint.setStartingPose(odometry.getPoseEstimate()); // Set starting pose

    distanceToTranslationTarget_mm = firstWaypoint.getSegmentDistance_mm();
    distanceToAngleTargetRadians = firstWaypoint.getDeltaHeadingRadians();

    // If more than one waypoint, loop through each waypoint and store starting positions
    // Also add the segment distance to the total distance
    if (waypointList.size() > 1) {
      for (int i = 1; i < waypointList.size(); i++) {
        Waypoint waypoint = waypointList.get(i);
        waypoint.setStartingPose(waypointList.get(i - 1).getTargetPose());
        distanceToTranslationTarget_mm += waypoint.getSegmentDistance_mm();
      }
    }
    /*
            // For movements to a field coordinate, we need to calculate the delta from the starting position
            allianceCFTargetPose = waypointList.get(currentSegmentBeingTraveled);
            if (commandType == COMMAND_TYPE.ABSOLUTE) {
                allianceCFDeltaPose = new Pose2D(
                        allianceCFTargetPose.x - allianceCFStartingPose.x,
                        allianceCFTargetPose.y - allianceCFStartingPose.y,
                        Field.enforceIMUHeadingRangeRadians(allianceCFTargetPose.heading - allianceCFStartingPose.heading)
                );
            }
    */
    // Set up translation motor profile
    translationMotionProfile = new MotionProfile(distanceToTranslationTarget_mm, v_max, a_max);

    // Set up rotation motion profile -- this is done per segment for multi-waypoint movements
    rotationMotionProfile =
        new MotionProfile(
            distanceToAngleTargetRadians, Math.toRadians(omega_max), Math.toRadians(alpha_max));
  }

  public COMMAND_CONTENTS getCommandType() {
    return commandContents;
  }

  public Pose2D getCurrentPIDControllerValues() {
    return PIDControllerValues;
  }

  @Override
  public boolean processCommand() {
    if (isCommandTimedOut()) {
      currentCommandPhase = COMMAND_COMPLETE;
      return false;
    }

    Pose2D currentAllianceCFPose =
        odometry.getPoseEstimate(); // Get current pose (alliance-centric)
    long elapsedTime = getElapsedCommandTime(Timer.TimerUnit.ms); // Get elapsed time
    Waypoint currentSegment = waypointList.get(currentSegmentBeingTraveled);

    // Obtain how far along the profile this command should be at this time
    double predictedLinearDistance = translationMotionProfile.getProfiledValue(elapsedTime);
    double percentOfSegmentLinearDistance;
    if (currentSegment.getSegmentDistance_mm() == 0.0) { // Special condition when this is 0
      percentOfSegmentLinearDistance = 1.0;
    } else {
      percentOfSegmentLinearDistance =
          (predictedLinearDistance - distanceOfCompletedSegments_mm)
              / currentSegment.getSegmentDistance_mm();
      if (percentOfSegmentLinearDistance > 1.0) {
        percentOfSegmentLinearDistance = 1.0;
      }
    }

    double predictedAngularDistance = rotationMotionProfile.getProfiledValue(elapsedTime);
    double percentOfTotalAngularDistance;
    if (distanceToAngleTargetRadians == 0.0) {
      percentOfTotalAngularDistance = 1.0;
    } else {
      percentOfTotalAngularDistance = predictedAngularDistance / distanceToAngleTargetRadians;
      if (percentOfTotalAngularDistance > 1.0) {
        percentOfTotalAngularDistance = 1.0;
      }
    }

    //        // If target counters in PID controllers have not been activated, and movement is >90%
    // of the distance
    //        // then activate the target counters
    ////        if (!translationTargetCounterActivated && (percentOfTotalLinearDistance > 0.90)) {
    //            XPIDController.activateTargetCounter();
    //            YPIDController.activateTargetCounter();
    //            translationTargetCounterActivated = true;
    ////        }
    ////        if (!rotationTargetCounterActivated && (percentOfTotalAngularDistance > 0.90)) {
    //            rotationPIDController.activateTargetCounter();
    //            rotationTargetCounterActivated = true;
    ////        }

    Pose2D currentSegmentStartingPose = currentSegment.getStartingPose();

    // Calculate the expected current location based on the above percentage of time elapsed on the
    // motion profile.
    Vector2D expectedCoords =
        new Vector2D(
            (currentSegment.getDeltaX() * percentOfSegmentLinearDistance)
                + currentSegmentStartingPose.x,
            (currentSegment.getDeltaY() * percentOfSegmentLinearDistance)
                + currentSegmentStartingPose.y);

    // Calculate the error by comparing the actual current location with the expected location
    double errorX = expectedCoords.x - currentAllianceCFPose.x;
    double errorY = expectedCoords.y - currentAllianceCFPose.y;
    //        Log.i(TAG, "expectedCoords x,y = " + expectedCoords.x + ", " + expectedCoords.y + ".
    // currentALlianceCFPose = " + currentAllianceCFPose.x + ", " + currentAllianceCFPose.y + ".
    // errorX=" + errorX + ", errorY=" + errorY);

    double currentMotionProfileTargetHeadingRadians =
        Field.addRadiansToIMUHeading(
            currentSegment.getStartingPose().heading,
            currentSegment.getDeltaHeadingRadians() * percentOfTotalAngularDistance);
    double errorHeadingRadians =
        Field.addRadiansToIMUHeading(
            currentMotionProfileTargetHeadingRadians, -currentAllianceCFPose.heading);

    // Update PIDController values
    PIDControllerValues.x = XPIDController.calculateNewControlValue(errorX);
    PIDControllerValues.y = YPIDController.calculateNewControlValue(errorY);
    PIDControllerValues.heading =
        rotationPIDController.calculateNewControlValue(errorHeadingRadians);

    //        Log.i(TAG, "PIDControllerValues x,y,hdg (deg) = " + PIDControllerValues.x + ", " +
    // PIDControllerValues.y + ", " + Math.toDegrees(PIDControllerValues.heading));
    // Check whether command is complete
    double actualDistanceToSegmentTarget =
        currentAllianceCFPose.getDistanceTo(currentSegment.getTargetPose());
    double actualDistanceToSegmentHeadingDegrees =
        Math.abs(
            Math.toDegrees(
                Field.addRadiansToIMUHeading(
                    currentSegment.getTargetPose().heading, -currentAllianceCFPose.heading)));
    double translationTargetTolerance;
    double rotationTargetTolerance;
    if (currentSegmentBeingTraveled < (waypointList.size() - 1)) {
      translationTargetTolerance = TRANSLATION_TARGET_TOLERANCE_WAYPOINT;
      rotationTargetTolerance = ROTATION_TARGET_TOLERANCE_WAYPOINT;
    } else {
      translationTargetTolerance = TRANSLATION_TARGET_TOLERANCE_FINAL;
      rotationTargetTolerance = ROTATION_TARGET_TOLERANCE_FINAL;
    }

    if ((actualDistanceToSegmentTarget <= translationTargetTolerance)
        && (actualDistanceToSegmentHeadingDegrees <= rotationTargetTolerance)) {
      isSegmentComplete = true;
    }

    // Log.i(TAG, "Traveling segment " + currentSegmentBeingTraveled + ". predictedLinearDist = " +
    // predictedLinearDistance + ". %SegmentDist = " + percentOfSegmentLinearDistance + ". Actual
    // Dist to Target = " + actualDistanceToSegmentTarget);
    // Log.i(TAG, "\tpredictedAngularDist = " + predictedAngularDistance + ". %SegmentDist = " +
    // percentOfTotalAngularDistance + ". Actual Heading to Target = " +
    // actualDistanceToSegmentHeadingDegrees);

    /*
            if (commandContents == COMMAND_CONTENTS.TRANSLATION) {
                if (XPIDController.isCommandComplete() && YPIDController.isCommandComplete()) {
                    isSegmentComplete = true;
                }
            }
            else if (commandContents == COMMAND_CONTENTS.ROTATION) {
                if (rotationPIDController.isCommandComplete()) {
                    isSegmentComplete = true;
                }
            }
            else {
                if (XPIDController.isCommandComplete() && YPIDController.isCommandComplete() && rotationPIDController.isCommandComplete()) {
                    isSegmentComplete = true;
                    Log.i("Test", "Setting isSegmentComplete = true");
                }
            }
    */
    if (isSegmentComplete) {

      currentSegmentBeingTraveled++;
      // Check if there are more segments to process
      if (currentSegmentBeingTraveled < waypointList.size()) {
        Log.i(Constants.TAG, "More segments to process");
        isSegmentComplete = false;
        distanceToAngleTargetRadians =
            waypointList.get(currentSegmentBeingTraveled).getDeltaHeadingRadians();

        XPIDController.resetErrorHistories();
        YPIDController.resetErrorHistories();
        rotationPIDController.resetErrorHistories();

        XPIDController.resetTargetCounter();
        YPIDController.resetTargetCounter();
        rotationPIDController.resetTargetCounter();

        // Set up new rotation motion profile -- this is done per segment for multi-waypoint
        // movements
        rotationMotionProfile =
            new MotionProfile(
                distanceToAngleTargetRadians, Math.toRadians(omega_max), Math.toRadians(alpha_max));

        // If this is the final segment, then change the target tolerance
        if (currentSegmentBeingTraveled == (waypointList.size() - 1)) {
          XPIDController.setTargetTolerance(TRANSLATION_TARGET_TOLERANCE_FINAL);
          YPIDController.setTargetTolerance(TRANSLATION_TARGET_TOLERANCE_FINAL);
          rotationPIDController.setTargetTolerance(ROTATION_TARGET_TOLERANCE_FINAL);
        }
      } else {
        currentCommandPhase = COMMAND_COMPLETE;
      }
    }
    return true;
  }
}
