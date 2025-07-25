package org.hexnibble.corelib.robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.PolarCoords;
import org.hexnibble.corelib.motion.MotorPowerSettings;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.pid.dtRotationPIDController;
import org.hexnibble.corelib.robot_system.CoreRobotSystem;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public class MecanumDrivetrain extends CoreRobotSystem
{
/*
  public native void onLoadJNI(BaseMotorWrapper LFMotor, BaseMotorWrapper RFMotor,
                               BaseMotorWrapper LBMotor, BaseMotorWrapper RBMotor);
  public native void onCloseJNI();
  public native void driveMecanumByCartesianENUCPP(
          double X, double Y, double spin,
          double offsetToConvertToFieldCentricHeadingDegrees);

  static {
    System.loadLibrary("CoreLib");
  }
*/
//  protected DriveController dtController;

  protected double dtManual_X; // X Joystick movement, range -1.0 (left) to +1.0 (right)
  protected double previousDTManual_X;
  protected double dtManual_Y; // Y Joystick movement, range -1.0 (backward) to +1.0 (forward)
  protected double previousDTManual_Y;
  protected double dtManual_Spin; // Spin, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
  protected double previousDTManual_Spin;
  private boolean dtManualMovementUpdated;
//  private double initialHeadingOnManualTranslationDegrees;
  private double currentIMUHeading;

  // Variables to hold motor objects.
  public enum WHEEL_MOTOR_NAME {
    LF, RF, LB, RB
  }

  private final WheelMotor motorLeftFront;
  private final WheelMotor motorRightFront;
  private final WheelMotor motorLeftBack;
  private final WheelMotor motorRightBack;

  // Variables to hold target power for each motor
  private final MotorPowerSettings targetMotorPowerSettings = new MotorPowerSettings();

  PIDSettings rotationPIDSettings =
          new PIDSettings(
                  ConfigFile.DRIVETRAIN_ROTATION_PID_Ks,
                  ConfigFile.DRIVETRAIN_ROTATION_PID_Kp,
                  ConfigFile.DRIVETRAIN_ROTATION_PID_Ki,
                  ConfigFile.DRIVETRAIN_ROTATION_PID_Kd);

  // Create PID Controllers
  protected dtRotationPIDController rotationPIDController = new dtRotationPIDController(
          rotationPIDSettings, Math.toRadians(2), dtRotationPIDController.ROTATION_DIRECTION.SHORTEST );

  // Minimum threshold for motor power to exceed to send a new motor command (compared to the
  // previously sent value)
  private final double MOTOR_POWER_THRESHOLD_FOR_NEW_COMMAND = 0.001;

  /**
   * Constructor for mecanum drivetrain.
   *
   * @param hwMap Hardware map
   * @param LFMotorName Name of the motor as specified in the driver station configuration
   * @param LFMotorRunDirection Motor run direction
   * @param LFEncoderType The type of encoder attached (Internal, Rev, GoBilda, etc) to the encoder
   *     port associated with this motor's power port.
   * @param LFEncoderDirection Encoder run direction (only used if an external encoder is specified)
   * @param RFMotorName Name of the motor as specified in the driver station configuration
   * @param RFMotorRunDirection Motor run direction
   * @param RFEncoderType The type of encoder attached (Internal, Rev, GoBilda, etc) to the encoder
   *     port associated with this motor's power port.
   * @param RFEncoderDirection Encoder run direction (only used if an external encoder is specified)
   * @param LBMotorName Name of the motor as specified in the driver station configuration
   * @param LBMotorRunDirection Motor run direction
   * @param LBEncoderType The type of encoder attached (Internal, Rev, GoBilda, etc) to the encoder
   *     port associated with this motor's power port.
   * @param LBEncoderDirection Encoder run direction (only used if an external encoder is specified)
   * @param RBMotorName Name of the motor as specified in the driver station configuration
   * @param RBMotorRunDirection Motor run direction
   * @param RBEncoderType The type of encoder attached (Internal, Rev, GoBilda, etc) to the encoder
   *     port associated with this motor's power port.
   * @param RBEncoderDirection Encoder run direction (only used if an external encoder is specified)
   * @param motorModel Motor model
   * @param runMode Motor run mode
   * @param extGearReduction External gear reduction
   * @param targetPositionTolerance Target position tolerance (counts)
   * @param wheelDiameterMM Diameter of the wheel attached to this motor. If the encoder port is
   *     being used for monitoring a separate wheel, then specify the diameter of the wheel attached
   *     to the encoder.
   */
  public MecanumDrivetrain(HardwareMap hwMap,
      String LFMotorName, DcMotor.Direction LFMotorRunDirection,
      BaseMotorWrapper.ENCODER LFEncoderType,
      DcMotor.Direction LFEncoderDirection,
      String RFMotorName, DcMotor.Direction RFMotorRunDirection,
      BaseMotorWrapper.ENCODER RFEncoderType, DcMotor.Direction RFEncoderDirection,
      String LBMotorName, DcMotor.Direction LBMotorRunDirection,
      BaseMotorWrapper.ENCODER LBEncoderType, DcMotor.Direction LBEncoderDirection,
      String RBMotorName, DcMotor.Direction RBMotorRunDirection,
      BaseMotorWrapper.ENCODER RBEncoderType, DcMotor.Direction RBEncoderDirection,
      BaseMotorWrapper.MOTOR_MODEL motorModel, DcMotor.RunMode runMode,
      double extGearReduction, int targetPositionTolerance, double wheelDiameterMM) {

    super(hwMap, "Drivetrain");

    Msg.log("MecanumDrivetrain", "Constructor", "Starting");

    motorLeftFront =
        new WheelMotor(hwMap, LFMotorName, motorModel, LFMotorRunDirection, runMode,
            LFEncoderType, LFEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
    motorRightFront =
        new WheelMotor(hwMap, RFMotorName, motorModel, RFMotorRunDirection, runMode,
            RFEncoderType, RFEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
    motorLeftBack =
        new WheelMotor(hwMap, LBMotorName, motorModel, LBMotorRunDirection, runMode,
            LBEncoderType, LBEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
    motorRightBack =
        new WheelMotor(hwMap, RBMotorName, motorModel, RBMotorRunDirection, runMode,
            RBEncoderType, RBEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);

//    dtController = new DriveController();

//    onLoadJNI(motorLeftFront, motorRightFront, motorLeftBack, motorRightBack);
    Msg.log("MecanumDrivetrain", "Constructor", "Ending");
  }

  /** Call this function to reinitialize motors to our set values when restarting an OpMode. */
  @Override
  public void resetSystem() {
    motorLeftFront.reset();
    motorRightFront.reset();
    motorLeftBack.reset();
    motorRightBack.reset();
    targetMotorPowerSettings.reset();
    dtManualMovementUpdated = false;
//    initialHeadingOnManualTranslationDegrees = 0.0;
    previousDTManual_X = 0.0;
    previousDTManual_Y = 0.0;
    previousDTManual_Spin = 0.0;
    currentIMUHeading = 0.0;
  }

  /**
   * Set the X value for a manual drivetrain movement.
   *
   * @param X Movement in X direction, range -1.0 (left) to +1.0 (right)
   */
  public void setDrivetrainManualMovement_X(double X) {
    previousDTManual_X = dtManual_X;
    dtManual_X = X;
    dtManualMovementUpdated = true;
  }

  /**
   * Set the Y value for a manual drivetrain movement.
   *
   * @param Y Movement in Y direction, range -1.0 (backward) to +1.0 (forward). Remember Y values
   *     need to be flipped when taken directly from the joystick.
   */
  public void setDrivetrainManualMovement_Y(double Y) {
    previousDTManual_Y = dtManual_Y;
    dtManual_Y = Y;
    dtManualMovementUpdated = true;
  }

  /**
   * Sets the spin for a manual drivetrain movement.
   *
   * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
   */
  public void setDrivetrainManualMovement_Spin(double spin) {
    previousDTManual_Spin = dtManual_Spin;
    dtManual_Spin = spin;
    dtManualMovementUpdated = true;
  }

  public WheelMotor getWheelMotorObject(WHEEL_MOTOR_NAME wheelMotorName) {
    return switch (wheelMotorName) {
      case LF -> motorLeftFront;
      case RF -> motorRightFront;
      case LB -> motorLeftBack;
      case RB -> motorRightBack;
    };
  }

  /** Brake drivetrain by setting all motors to 0 power. */
  public void brakeDrivetrain() {
    motorLeftFront.setPower(0.0);
    motorRightFront.setPower(0.0);
    motorLeftBack.setPower(0.0);
    motorRightBack.setPower(0.0);
  }

  public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
    motorLeftFront.setBrakeMode(brakeMode);
    motorRightFront.setBrakeMode(brakeMode);
    motorLeftBack.setBrakeMode(brakeMode);
    motorRightBack.setBrakeMode(brakeMode);
  }

  public double getMotorCurrent(@NonNull WHEEL_MOTOR_NAME motorName) {
    return switch (motorName) {
      case LF -> motorLeftFront.getMotorCurrent(CurrentUnit.MILLIAMPS);
      case RF -> motorRightFront.getMotorCurrent(CurrentUnit.MILLIAMPS);
      case LB -> motorLeftBack.getMotorCurrent(CurrentUnit.MILLIAMPS);
      case RB -> motorRightBack.getMotorCurrent(CurrentUnit.MILLIAMPS);
    };
  }

  public double getMotorPower(@NonNull WHEEL_MOTOR_NAME motorName) {
    return switch (motorName) {
      case LF -> motorLeftFront.getMotorPower();
      case RF -> motorRightFront.getMotorPower();
      case LB -> motorLeftBack.getMotorPower();
      case RB -> motorRightBack.getMotorPower();
    };
  }

  /**
   * Move mecanum drivetrain based on robot CF Cartesian coordinates using ENU (East-North_Up)
   * convention. This can be robot-centric joystick X/Y and spin values. A heading offset can be
   * provided to convert to field-centric coordinates This function converts the Cartesion
   * coordinates to polar coordinates and then passes it to the actual drive function.
   *
   * @param X Movement in X direction, range -1.0 (left) to +1.0 (right)
   * @param Y Movement in Y direction, range -1.0 (backward) to +1.0 (forward). Remember Y values
   *     need to be flipped when taken directly from the joystick.
   * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
   * @param offsetToConvertToFieldCentricHeadingDegrees Offset to subtract to convert robot's zero
   *     to field's zero heading
   */
  public void driveMecanumByCartesianENU(
      double X, double Y, double spin, double offsetToConvertToFieldCentricHeadingDegrees) {

    final boolean useCPP = false;

    if (useCPP) {
      // CPP version
      // This does all the calculations and sends the motor commands
/*
      long startTime = System.nanoTime();
      driveMecanumByCartesianENUCPP(X, Y, spin, offsetToConvertToFieldCentricHeadingDegrees);
      Msg.log("MecanumDrivetrain", "driveMecanumByRobotPolarHeading", "Time for CPP version= "
              + (System.nanoTime() - startTime)/1000 + " us\n");
*/
    }
    else {
      // Java version

      long startTime = System.nanoTime();
      // Convert X and Y (robot-centric) to polar coordinates (robot-centric)
      // Remember 0 degrees lies on x-axis in polar coordinates
      PolarCoords coords = Field.cartesianToPolarCoords(X, Y);

      // Check r to make it zero if it is effectively too small. r has to be >=0 from the above fx
      if (coords.r < MOTOR_POWER_THRESHOLD_FOR_NEW_COMMAND) {
        coords.r = 0.0;
      }

      // Adjust heading for field-centric coordinates. No change would occur if offset is 0
      coords.theta -= Math.toRadians(offsetToConvertToFieldCentricHeadingDegrees);

      calculateMotorPowers(coords.r, coords.theta, spin, targetMotorPowerSettings);
//      setMotorPowers(targetMotorPowerSettings);
//      Msg.log("MecanumDrivetrain", "driveMecanumByRobotPolarHeading", "Time for Java version= "
//              + (System.nanoTime() - startTime)/1000 + " us");
    }
  }

  /**
   * Move mecanum drivetrain based on robot-centric compass heading
   *
   * @param headingDegrees Robot-centric compass heading (0 - 360 degrees), where 0 deg is forward
   * @param motorPower Motor power for translation (must be a positive value from 0.0 - 1.0)
   * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
   */
  public void driveMecanumByRobotCompassHeading(
      double headingDegrees, float motorPower, double spin) {
    // Convert robot-centric compass heading (0-deg is on y-axis) to robot-centric polar heading
    // (0-deg is on x-axis)
    // Also convert to radians
    double theta = Math.toRadians((360.0 - headingDegrees) + 90.0);

    driveMecanumByRobotPolarHeading(motorPower, theta, spin);
  }

  /**
   * Calculate motor powers for each wheel motor to move mecanum drivetrain based on robot-centric
   * polar coordinates. An angle (heading) of 0 is to the right.
   *
   * @param r Magnitude of movement (0 - 1).
   * @param theta Heading (angle of movement) in radians using polar coordinates. 0 is to the right
   *     of the robot. Value will be corrected to ensure it is in range 0 - 2pi
   * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
   */
  private void driveMecanumByRobotPolarHeading(double r, double theta, double spin) {
    calculateMotorPowers(r, theta, spin, targetMotorPowerSettings);
    setMotorPowers(targetMotorPowerSettings);
  }

  /**
   *
   * @param r Magnitude of movement (0 - 1). Value will be clamped.
   * @param theta Heading (angle of movement) in radians using polar coordinates. 0 is to the right
   *     of the robot. Value will be corrected to ensure it is in range 0 - 2pi
   * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule. Value will be clamped
   * @param mPowers MotorPowerSettings structure reference to store the calculated motor powers.
   */
  private void calculateMotorPowers(double r, double theta, double spin, MotorPowerSettings mPowers) {
    // Calculate left and right motor power values based on magnitude of r
    double targetLeftMotorPower = 0.0;
    double targetRightMotorPower = 0.0;

    // Motors will only be given translation power if r > 0
    r = Math.clamp(r, 0.0, 1.0);
    if (r > 0.0) {
      theta = Field.enforceHeadingRangeTauRadians(theta);

      if (theta <= Field.DEG_090_IN_RADS) {
        targetLeftMotorPower = r;
        targetRightMotorPower = -r * Math.cos(theta * 2.0);
      }
      else if (theta < Math.PI) {
        targetLeftMotorPower = -r * Math.cos(theta * 2.0);
        targetRightMotorPower = r;
      }
      else if (theta <= Field.DEG_270_IN_RADS) {
        targetLeftMotorPower = -r;
        targetRightMotorPower = r * Math.cos(theta * 2.0);
      }
      else { // theta is >270.0 to <360.0 degrees
        targetLeftMotorPower = r * Math.cos(theta * 2.0);
        targetRightMotorPower = -r;
      }
    }

    // Combine translation and spin movements to obtain normalized values so that each movement type
    // receives equal weight.
    // This process assumes the largest r value will be 1. This works with a joystick having
    // circular movement boundaries.
    // But a joystick with square boundaries or PID controller values with r values exceeding 1
    // will be clipped by the motors. This effect is most troublesome in the corner areas (when
    // thinking in terms of joystick movement).
    spin = Math.clamp(spin, -1.0, 1.0);
    final double spinNormalizer = (2.0 - Math.abs(spin)) / 2.0;
    double absMaxTranslationMotorPower =
            Math.max(Math.abs(targetLeftMotorPower), Math.abs(targetRightMotorPower));
    double normalizedTargetLeftMotorPower = targetLeftMotorPower * spinNormalizer;
    double normalizedTargetRightMotorPower = targetRightMotorPower * spinNormalizer;
    double normalizedSpinMotorPower = spin * (2.0 - absMaxTranslationMotorPower) / 2.0;

    // Spin is achieved by adding/subtracting power to motors on opposite sides of the robot
    // Subtract from left side motors
    mPowers.LF = normalizedTargetLeftMotorPower - normalizedSpinMotorPower;
    mPowers.LB = normalizedTargetRightMotorPower - normalizedSpinMotorPower;
    // Add to right side motors
    mPowers.RF = normalizedTargetRightMotorPower + normalizedSpinMotorPower;
    mPowers.RB = normalizedTargetLeftMotorPower + normalizedSpinMotorPower;
  }

  private void setMotorPowers(MotorPowerSettings targetMPowers) {
    motorLeftFront.setPowerNoClamping(targetMPowers.LF);
    motorRightFront.setPowerNoClamping(targetMPowers.RF);
    motorLeftBack.setPowerNoClamping(targetMPowers.LB);
    motorRightBack.setPowerNoClamping(targetMPowers.RB);
  }

  public void setMotorPowers(double LF, double RF, double LB, double RB) {
    motorLeftFront.setPowerNoClamping(LF);
    motorRightFront.setPowerNoClamping(RF);
    motorLeftBack.setPowerNoClamping(LB);
    motorRightBack.setPowerNoClamping(RB);
  }

  public void setMotorRunMode(WHEEL_MOTOR_NAME motorName, DcMotor.RunMode runMode) {
    WheelMotor motor =
        switch (motorName) {
          case LF -> motorLeftFront;
          case LB -> motorLeftBack;
          case RF -> motorRightFront;
          case RB -> motorRightBack;
        };
    motor.setRunMode(runMode);
  }

  public void setMotorPower(WHEEL_MOTOR_NAME motorName, double power) {
    WheelMotor motor =
        switch (motorName) {
          case LF -> motorLeftFront;
          case LB -> motorLeftBack;
          case RF -> motorRightFront;
          case RB -> motorRightBack;
        };
    motor.setPower(power);
  }

  public void setCurrentIMUHeading(double currentIMUHeading) {
    this.currentIMUHeading = currentIMUHeading;
  }

  @Override
  public void processCommands() {
//    dtController.processPath();
    double spinValue = dtManual_Spin;

    if (dtManualMovementUpdated) {
      clearSystemRCList();

/*
      // If a new translation movement is starting OR a manual spin is ending, save the heading
      if (((previousDTManual_X == 0.0) && (dtManual_X != 0.0))
          || ((previousDTManual_Y == 0.0) && (dtManual_Y != 0.0))
          || ((previousDTManual_Spin != 0.0) && (dtManual_Spin == 0.0))) {

        Msg.log("MecanumDrivetrain", "processCommands", "Saving start heading=" + currentIMUHeading);
        initialHeadingOnManualTranslationDegrees = currentIMUHeading;
      }

      // If translating without spin, lock the heading to the saved value so the robot's heading
      // does not drift.
//      Msg.log("MecanumDrivetrain", "processCommands", "dtManual_Spin=" + dtManual_Spin);

      if ((dtManual_Spin == 0.0)
          && ((dtManual_X != 0.0) || (dtManual_Y != 0.0))) {
        Msg.log("MecanumDrivetrain", "processCommands",
                "No spin so subtracting current IMU=" + currentIMUHeading + " from startHdg=" + initialHeadingOnManualTranslationDegrees);

        double errorHdgRadians = Field.addRadiansToIMUHeading(-Math.toRadians(currentIMUHeading), Math.toRadians(initialHeadingOnManualTranslationDegrees));
        spinValue = rotationPIDController.calculateNewControlValue(errorHdgRadians);
        Msg.log("MecanumDrivetrain", "processCommands", "error=" + Math.toDegrees(errorHdgRadians)
                + " spin=" + spinValue);
      }
*/

      driveMecanumByCartesianENU(dtManual_X, dtManual_Y, spinValue, currentIMUHeading);
      setMotorPowers(targetMotorPowerSettings);
      dtManualMovementUpdated = false;
    }
//    else if (!isSystemRCListEmpty()) {
////      Msg.log("MecanumDrivetrain", "processCommands", "here3");
//      if (systemRCList.get(0).processRC()) {
//
//        systemRCList.remove(0);
//      }
//
//      setMotorPowers(targetMotorPowerSettings);
//    }
  }
}
