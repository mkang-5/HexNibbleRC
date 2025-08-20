package org.hexnibble.corelib.motion;

import androidx.annotation.NonNull;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.path.CorePath;
import org.hexnibble.corelib.motion.path.Line;
import org.hexnibble.corelib.motion.path.PathChain;
import org.hexnibble.corelib.motion.path.Spin;
import org.hexnibble.corelib.motion.pid.PIDController;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.pid.dtRotationPIDController;
import org.hexnibble.corelib.robot.drivetrain.BaseDrivetrain;

/*  The DriveController holds a drivetrain movement PathChain and directs the processes to move
each path. This includes the PID controllers.

 */
public class DriveController {
  public enum STATUS {IDLE, HOLDING, FOLLOWING_PATH}
  private STATUS status;
  private final BaseDrivetrain dt;
  private PathChain pathChain;
  private int pathChainIndex;
  private CorePath currentPath;
  private Pose2D holdPose;        // Pose to hold, with IMU heading in radians

//  private boolean isIdle;

  // Set up translation and rotation PID Controllers
  PIDSettings translationPIDSettings =
      new PIDSettings(
          ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ks,
          ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kp,
          ConfigFile.DRIVETRAIN_TRANSLATION_PID_Ki,
          ConfigFile.DRIVETRAIN_TRANSLATION_PID_Kd);
  PIDSettings rotationPIDSettings =
      new PIDSettings(
          ConfigFile.DRIVETRAIN_ROTATION_PID_Ks,
          ConfigFile.DRIVETRAIN_ROTATION_PID_Kp,
          ConfigFile.DRIVETRAIN_ROTATION_PID_Ki,
          ConfigFile.DRIVETRAIN_ROTATION_PID_Kd);

  // Create PID Controllers
  protected PIDController xPIDController =
        new PIDController(translationPIDSettings, 5);
  protected PIDController yPIDController =
        new PIDController(translationPIDSettings, 5);
  protected dtRotationPIDController rotationPIDController =
        new dtRotationPIDController(rotationPIDSettings, Math.toRadians(2.0),
              dtRotationPIDController.ROTATION_DIRECTION.SHORTEST );

  public DriveController(BaseDrivetrain dt) {
    this.dt = dt;
    status = STATUS.IDLE;
  }

  /**
   * This function is called by a DrivetrainRC at the start of the command to begin the provided
   * pathChain (trajectory).
   * @param pathChain The requested trajectory
   */
  public void startTrajectory(@NonNull PathChain pathChain) {
    this.pathChain = pathChain;
    pathChainIndex = 0;
    currentPath = this.pathChain.getPath(pathChainIndex);
    status = STATUS.FOLLOWING_PATH;

    xPIDController.reset();
    yPIDController.reset();
    rotationPIDController.reset();

    Msg.log(getClass().getSimpleName(), "startTrajectory", "Starting a new trajectory with pathChain size=" + pathChain.size());
  }

  /**
   * Advance the active path on the pathChain, if one exists.
   * @param currentPose Current pose with IMU Heading in radians
   */
  private void getNextPath(Pose2D currentPose) {
    pathChainIndex++;
    if (pathChainIndex < pathChain.size()) {
      currentPath = pathChain.getPath(pathChainIndex);
    }
    else {
      if (pathChain.getHoldPosition()) {
        status = STATUS.HOLDING;
        setHoldPose(pathChain.getPath(pathChainIndex - 1).getTargetPose());
//        setHoldPose(currentPose);
      }
      currentPath = null;
      pathChain = null;
    }
  }

  /**
   *
   * @param currentPose Current pose, with IMU Heading in radians.
   */
  public void calculatePath(final Pose2D currentPose) {

//    currentPath.getClosestInterpolatedTValue(currentPose);

    // Update heading PIDController values
    double errorHeadingRadians = currentPath.getHeadingError(currentPose.heading);
    double headingControlValue = rotationPIDController.calculateNewControlValue(errorHeadingRadians);
    dt.setDtAutoMovementSpin(headingControlValue);

    // Process lines
    if (!(currentPath instanceof Spin)) {
      // Update PIDController values
//      double errorX = currentPose.x - currentPath.getTargetPose().x;
      double errorX = ((Line) currentPath).getXError(currentPose);
      double XControlValue = xPIDController.calculateNewControlValue(errorX);
      dt.setDtAutoMovementX(XControlValue);

//      double errorY = currentPose.y - currentPath.getTargetPose().y;
      double errorY = ((Line) currentPath).getXError(currentPose);
      double YControlValue = yPIDController.calculateNewControlValue(errorY);
      dt.setDtAutoMovementY(-YControlValue);

    }

    if (currentPath.isPathComplete(currentPose)) {
      getNextPath(currentPose);
    }
  }

  /**
   * Set the pose for the robot to hold.
   * @param holdPose Pose to hold, with IMU heading in radians
   */
  private void setHoldPose(Pose2D holdPose) {
    this.holdPose = new Pose2D(holdPose);
    xPIDController.reset();
    yPIDController.reset();
    rotationPIDController.reset();

    Msg.log(getClass().getSimpleName(), "setHoldPose", "Setting hold pose to: " + this.holdPose.x + ", " + this.holdPose.y + ", " + Math.toDegrees(this.holdPose.heading) + " deg");
  }

  /**
   * This function is used to hold a pose when no paths are active (if requested to hold).
   * @param currentPose Current pose, with IMU Heading in radians.
   */
  private void calculatePathToHoldPose(Pose2D currentPose) {
    double errorTranslationXmm = currentPose.x - holdPose.x;
    Msg.log(getClass().getSimpleName(), "calculatePathToHoldPose", "holdPose.x=" + holdPose.x + ", currentPose.x=" + currentPose.x + ", errorX=" + errorTranslationXmm);

    double errorTranslationYmm = currentPose.y - holdPose.y;
    Msg.log(getClass().getSimpleName(), "calculatePathToHoldPose", "holdPose.y=" + holdPose.y + ", currentPose.y=" + currentPose.y + ", errorY=" + errorTranslationYmm);

    double errorHeadingRadians = Field.addRadiansToIMUHeading(holdPose.heading, -currentPose.heading);
    Msg.log(getClass().getSimpleName(), "calculatePathToHoldPose", "holdPose.hdg (deg)=" + Math.toDegrees(holdPose.heading) + "currentPose.hdg (deg)=" + Math.toDegrees(currentPose.heading) + "errorHeadingDegrees=" + Math.toDegrees(errorHeadingRadians));

    // Update PIDController values
    double xControlValue = xPIDController.calculateNewControlValue(errorTranslationXmm);
    double yControlValue = yPIDController.calculateNewControlValue(errorTranslationYmm);
    double headingControlValue = rotationPIDController.calculateNewControlValue(errorHeadingRadians);

    dt.setDtAutoMovementX(xControlValue);
    dt.setDtAutoMovementY(-yControlValue);          // Flip the sign for y joystick
    dt.setDtAutoMovementSpin(headingControlValue);

    Msg.log(getClass().getSimpleName(), "calculatePathToHoldPose", "PIDx=" + xControlValue + ", PIDy (unflipped)=" + yControlValue + ", PIDHdg=" + headingControlValue);
  }

  public STATUS getStatus() {
    return status;
  }

  public void setHoldPositionOff() {
    holdPose = null;
    status = STATUS.IDLE;
  }

  /**
   * This function is called by the RCController each loop to run this driveController.
   * @param currentPose Current pose, with IMU heading in radians
   */
  public void processPath(Pose2D currentPose) {
    if (dt.isManualMovementUpdated()) {
//      Msg.log(getClass().getSimpleName(), "processPath", "isManualMovementUpdated = true");
      status = STATUS.IDLE;

      // Remove any auto movements if a manual movement is occurring
      currentPath = null;
      pathChain = null;

      Msg.log(getClass().getSimpleName(), "processPath", "currentPose Heading=" + Math.toDegrees(currentPose.heading));

      dt.driveByRobotCartesianENU(dt.getDtManualMovementX(), dt.getDtManualMovementY(),
            dt.getDtManualMovementSpin(), Math.toDegrees(currentPose.heading));
    }
    else {
//      Msg.log(getClass().getSimpleName(), "processPath", "isManualMovementUpdated = false");

      // If there is a path, then drive it. Otherwise, check if the robot should hold position.
      if (currentPath != null) {
//        Msg.log(getClass().getSimpleName(), "processPath", "currentPath != null");
        calculatePath(currentPose);
        dt.driveByRobotCartesianENU(dt.getDtAutoMovementX(), dt.getDtAutoMovementY(),
              dt.getDtAutoMovementSpin(), Math.toDegrees(currentPose.heading));
      }
      else if (status == STATUS.HOLDING) {
        calculatePathToHoldPose(currentPose);
        dt.driveByRobotCartesianENU(dt.getDtAutoMovementX(), dt.getDtAutoMovementY(),
              dt.getDtAutoMovementSpin(), Math.toDegrees(currentPose.heading));
        Msg.log(getClass().getSimpleName(), "processPath", "status == HOLDING");
      }
    }
  }
}
