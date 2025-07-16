package org.hexnibble.corelib.motion;

import androidx.annotation.NonNull;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.path.CorePath;
import org.hexnibble.corelib.motion.path.PathChain;
import org.hexnibble.corelib.motion.pid.PIDController;
import org.hexnibble.corelib.motion.pid.PIDSettings;

public class DriveController {
  private PathChain pathChain;
  private int pathChainIndex;
  private CorePath currentPath;

  private boolean isIdle;

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
      new PIDController(translationPIDSettings, 5); // Kp = 0.01

  //    protected dtRotationPIDController rotationPIDController = new dtRotationPIDController(
  //            rotationPIDSettings, Math.toRadians(2), this.rotationDirection); // Kp = 0.01

  /**
   * This function is called to start a trajectory.
   * @param pathChain
   */
  public void startTrajectory(@NonNull PathChain pathChain) {
    this.pathChain = pathChain;
    currentPath = this.pathChain.getPath(0);
    pathChainIndex = 0;
    isIdle = false;
  }

  public void calculatePath() { // Probably needs the current pose as an argument
    Pose2D currentPose = new Pose2D(); // Placeholder
    currentPath.getClosestInterpolatedTValue(currentPose);

    // Calculate the joystick values needed (x/y/spin)
    // This needs a target
  }

  public boolean isIdle() {
    return isIdle;
  }

  /**
   * This function should be called each loop to calculate the PID outputs to run the path.
   * @param currentIMUHeading
   * @param targetMotorPowerSettings
   */
  public void processPath(double currentIMUHeading, MotorPowerSettings targetMotorPowerSettings) {



    double targetHdgRadians = 0.0;
    double errorHdgRadians = Field.addRadiansToIMUHeading(targetHdgRadians, -currentIMUHeading);
    //        double spinValue = rotationPIDController.calculateNewControlValue(errorHdgRadians);
  }
}
