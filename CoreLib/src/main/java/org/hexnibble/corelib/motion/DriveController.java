package org.hexnibble.corelib.motion;

import androidx.annotation.NonNull;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.path.CorePath;
import org.hexnibble.corelib.motion.path.PathChain;
import org.hexnibble.corelib.motion.path.Spin;
import org.hexnibble.corelib.motion.pid.PIDController;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.pid.dtRotationPIDController;
import org.hexnibble.corelib.robot.drivetrain.BaseDrivetrain;

// This is similar to the Pedro Follower
public class DriveController {
  public enum STATUS {IDLE, HOLDING, FOLLOWING_PATH}
  private STATUS status;
  private BaseDrivetrain dt;
  private PathChain pathChain;
  private int pathChainIndex;
  private CorePath currentPath;
//  private boolean holdPosition;
  private Pose2D holdPose;

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
  protected PIDController translationPIDController =
        new PIDController(translationPIDSettings, 5);
  protected dtRotationPIDController rotationPIDController =
        new dtRotationPIDController(rotationPIDSettings, Math.toRadians(2.0),
              dtRotationPIDController.ROTATION_DIRECTION.SHORTEST );

  public DriveController(BaseDrivetrain dt) {
    this.dt = dt;
    status = STATUS.IDLE;
  }

  /**
   * This function is called to start a trajectory.
   * @param pathChain
   */
  public void startTrajectory(@NonNull PathChain pathChain) {
    this.pathChain = pathChain;
    pathChainIndex = 0;
    currentPath = this.pathChain.getPath(pathChainIndex);
    status = STATUS.FOLLOWING_PATH;
//    isIdle = false;
//    holdPosition = pathChain.getHoldPosition();

    translationPIDController.reset();
    rotationPIDController.reset();

    Msg.log(getClass().getSimpleName(), "startTrajectory", "pathChain size=" + pathChain.size());
  }

  private void getNextPath(double currentIMUHeading) {
    pathChainIndex++;
    if (pathChainIndex < pathChain.size()) {
      currentPath = pathChain.getPath(pathChainIndex);
    }
    else {
      if (pathChain.getHoldPosition()) {
        status = STATUS.HOLDING;
        setHoldPose(currentIMUHeading);
      }
      currentPath = null;
      pathChain = null;
//      isIdle = true;
    }
  }

  public void calculatePath(double currentIMUHeading) { // Probably needs the current pose as an argument
//    Pose2D currentPose = new Pose2D(); // Placeholder
//    currentPath.getClosestInterpolatedTValue(currentPose);

    // Calculate the joystick values needed (x/y/spin)
    // This needs a target

//    if (currentPath != null) {
      // Process isolated spins
      if (currentPath instanceof Spin) {

        Msg.log(getClass().getSimpleName(), "calculatePath", "calculatePath called 3");
        double errorHeadingDegrees =
              Field.addDegreesToIMUHeading(
//              Field.addRadiansToIMUHeading(
                    ((Spin) currentPath).getTargetHeading(), -currentIMUHeading);

        // Update PIDController values
        double headingControlValue = rotationPIDController.calculateNewControlValue(Math.toRadians(errorHeadingDegrees));
        dt.setDtAutoMovementSpin(headingControlValue);

        Msg.log(getClass().getSimpleName(), "calculatePath", "errorHdgDeg=" + errorHeadingDegrees + ", CtrlValue=" + headingControlValue);

        if (rotationPIDController.isCommandComplete()) {
          Msg.log(getClass().getSimpleName(), "calculatePath", "errorHdg=" + errorHeadingDegrees + ", rotationPIDController.isCommandComplete = true");
          getNextPath(currentIMUHeading);
        }
      }
//    }
  }

  private void setHoldPose(double currentIMUHeading) {
    holdPose = new Pose2D(0.0, 0.0, currentIMUHeading);
    translationPIDController.reset();
    rotationPIDController.reset();
  }

  private void calculatePathToHoldPose(double currentIMUHeading) {
    double errorHeadingDegrees =
          Field.addDegreesToIMUHeading(
//              Field.addRadiansToIMUHeading(
                holdPose.heading, -currentIMUHeading);

    // Update PIDController values
    double headingControlValue = rotationPIDController.calculateNewControlValue(Math.toRadians(errorHeadingDegrees));
    dt.setDtAutoMovementSpin(headingControlValue);
  }

//  public boolean isIdle() {
//    return isIdle;
//  }

  public STATUS getStatus() {
    return status;
  }

  public void setHoldPositionOff() {
//    holdPosition = false;
    status = STATUS.IDLE;
  }

  /**
   * This function should be called each loop to calculate the PID outputs to run the path.
   * @param currentIMUHeading Current IMU heading in degrees
   */
  public void processPath(double currentIMUHeading) {

    if (dt.isManualMovementUpdated()) {
      Msg.log(getClass().getSimpleName(), "processPath", "isManualMovementUpdated = true");
      status = STATUS.IDLE;
      // Remove any auto movements if a manual movement is occurring
      if (currentPath != null) {
        currentPath = null;
//        isIdle = true;
//        holdPosition = false;
      }

      dt.driveByRobotCartesianENU(dt.getDtManualMovementX(), dt.getDtManualMovementY(), dt.getDtManualMovementSpin(),
          currentIMUHeading);
    }
    else {
      Msg.log(getClass().getSimpleName(), "processPath", "isManualMovementUpdated = false");

      if (currentPath != null) {
        Msg.log(getClass().getSimpleName(), "processPath", "currentPath != null");
        calculatePath(currentIMUHeading);
        dt.driveByRobotCartesianENU(dt.getDtAutoMovementX(), dt.getDtAutoMovementY(), dt.getDtAutoMovementSpin(),
              currentIMUHeading);
      }
      else if (status == STATUS.HOLDING) {
        calculatePathToHoldPose(currentIMUHeading);
        dt.driveByRobotCartesianENU(dt.getDtAutoMovementX(), dt.getDtAutoMovementY(), dt.getDtAutoMovementSpin(),
              currentIMUHeading);
        Msg.log(getClass().getSimpleName(), "processPath", "status == HOLDING");
      }
    }


  }
}
