package org.hexnibble.corelib.robot.drivetrain;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.hexnibble.corelib.commands.rc.DrivetrainRC;
import org.hexnibble.corelib.commands.rc.RCController;
import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.PolarCoords;
import org.hexnibble.corelib.motion.MotorPowerSettings;
import org.hexnibble.corelib.motion.path.CorePath;
import org.hexnibble.corelib.motion.path.PathChain;
import org.hexnibble.corelib.motion.path.Spin;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.pid.dtRotationPIDController;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public class MecanumDrivetrain extends BaseDrivetrain
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

  // Variables to hold motor objects.
  public enum WHEEL_MODULE_NAME {
    LF, RF, LB, RB
  }

  private final WheelModule moduleLeftFront;
  private final WheelModule moduleRightFront;
  private final WheelModule moduleLeftBack;
  private final WheelModule moduleRightBack;

  // Variables to hold target power for each motor
  private final MotorPowerSettings targetMotorPowerSettings = new MotorPowerSettings();


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
  public MecanumDrivetrain(@NonNull HardwareMap hwMap,
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

    super(hwMap);

    Msg.log("MecanumDrivetrain", "Constructor", "Starting");

    moduleLeftFront =
        new WheelModule(hwMap, LFMotorName, motorModel, LFMotorRunDirection, runMode,
            LFEncoderType, LFEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
    moduleRightFront =
        new WheelModule(hwMap, RFMotorName, motorModel, RFMotorRunDirection, runMode,
            RFEncoderType, RFEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
    moduleLeftBack =
        new WheelModule(hwMap, LBMotorName, motorModel, LBMotorRunDirection, runMode,
            LBEncoderType, LBEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
    moduleRightBack =
        new WheelModule(hwMap, RBMotorName, motorModel, RBMotorRunDirection, runMode,
            RBEncoderType, RBEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);

//    onLoadJNI(motorLeftFront, motorRightFront, motorLeftBack, motorRightBack);
    Msg.log("MecanumDrivetrain", "Constructor", "Ending");
  }

  /** Call this function to reinitialize motors to our set values when restarting an OpMode. */
  @Override
  public void resetSystem() {
    super.resetSystem();

    moduleLeftFront.reset();
    moduleRightFront.reset();
    moduleLeftBack.reset();
    moduleRightBack.reset();

    targetMotorPowerSettings.reset();
  }

  public WheelMotor getWheelMotorObject(WHEEL_MODULE_NAME wheelModuleName) {
    return switch (wheelModuleName) {
      case LF -> moduleLeftFront.getWheelMotorObject();
      case RF -> moduleRightFront.getWheelMotorObject();
      case LB -> moduleLeftBack.getWheelMotorObject();
      case RB -> moduleRightBack.getWheelMotorObject();
    };
  }

  /** Brake drivetrain by setting all motors to 0 power. */
  @Override
  public void brakeDrivetrain() {
    moduleLeftFront.setMotorPower(0.0);
    moduleRightFront.setMotorPower(0.0);
    moduleLeftBack.setMotorPower(0.0);
    moduleRightBack.setMotorPower(0.0);
  }

  @Override
  public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
    moduleLeftFront.setBrakeMode(brakeMode);
    moduleRightFront.setBrakeMode(brakeMode);
    moduleLeftBack.setBrakeMode(brakeMode);
    moduleRightBack.setBrakeMode(brakeMode);
  }

  public double getMotorCurrent(@NonNull WHEEL_MODULE_NAME moduleName) {
    return switch (moduleName) {
      case LF -> moduleLeftFront.getMotorCurrent(CurrentUnit.MILLIAMPS);
      case RF -> moduleRightFront.getMotorCurrent(CurrentUnit.MILLIAMPS);
      case LB -> moduleLeftBack.getMotorCurrent(CurrentUnit.MILLIAMPS);
      case RB -> moduleRightBack.getMotorCurrent(CurrentUnit.MILLIAMPS);
    };
  }

  public double getMotorPower(@NonNull WHEEL_MODULE_NAME moduleName) {
    return switch (moduleName) {
      case LF -> moduleLeftFront.getMotorPower();
      case RF -> moduleRightFront.getMotorPower();
      case LB -> moduleLeftBack.getMotorPower();
      case RB -> moduleRightBack.getMotorPower();
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
  @Override
  public void driveByRobotCartesianENU(
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

//      long startTime = System.nanoTime();
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
      setMotorPowers(targetMotorPowerSettings);
      dtManualMovementUpdated = false;
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
//  @Override
//  public void driveByRobotCompassHeading(double headingDegrees, float motorPower, double spin) {
//    // Convert robot-centric compass heading (0-deg is on y-axis) to robot-centric polar heading
//    // (0-deg is on x-axis)
//    // Also convert to radians
//    double theta = Math.toRadians((360.0 - headingDegrees) + 90.0);
//
//    driveMecanumByRobotPolarHeading(motorPower, theta, spin);
//  }

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
   * Calculate the motor powers needed to move the drivetrain the desired settings.
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
    moduleLeftFront.setMotorPowerNoClamping(targetMPowers.LF);
    moduleRightFront.setMotorPowerNoClamping(targetMPowers.RF);
    moduleLeftBack.setMotorPowerNoClamping(targetMPowers.LB);
    moduleRightBack.setMotorPowerNoClamping(targetMPowers.RB);
  }

  public void setMotorPowers(double LF, double RF, double LB, double RB) {
    moduleLeftFront.setMotorPowerNoClamping(LF);
    moduleRightFront.setMotorPowerNoClamping(RF);
    moduleLeftBack.setMotorPowerNoClamping(LB);
    moduleRightBack.setMotorPowerNoClamping(RB);
  }

  public void setMotorRunMode(WHEEL_MODULE_NAME moduleName, DcMotor.RunMode runMode) {
    WheelModule module =
        switch (moduleName) {
          case LF -> moduleLeftFront;
          case LB -> moduleLeftBack;
          case RF -> moduleRightFront;
          case RB -> moduleRightBack;
        };
    module.setMotorRunMode(runMode);
  }

  public void setMotorPower(WHEEL_MODULE_NAME moduleName, double power) {
    WheelModule module =
        switch (moduleName) {
          case LF -> moduleLeftFront;
          case LB -> moduleLeftBack;
          case RF -> moduleRightFront;
          case RB -> moduleRightBack;
        };
    module.setMotorPower(power);
  }

  public void qSpinTurnToNearest45(RCController rcController, double currentIMUHeadingDegrees,
                                   CorePath.ROTATION_DIRECTION rotationDirection) {

    final double targetTolerance = 2.5; // Must be >0.0
    double deltaIMUHeadingDegrees;

    Msg.log(getClass().getSimpleName(), "qSpinTurnToNearest45", "Queueing " + rotationDirection + " spin turn to nearest 45 degrees");

    if (rotationDirection == CorePath.ROTATION_DIRECTION.CLOCKWISE) {
      deltaIMUHeadingDegrees =
            (Math.ceil((currentIMUHeadingDegrees - targetTolerance) / 45.0) * 45.0)
                  - 45.0
                  - currentIMUHeadingDegrees;
    }
    else {
      deltaIMUHeadingDegrees =
            (Math.floor((currentIMUHeadingDegrees + targetTolerance) / 45.0) * 45.0)
                  + 45.0
                  - currentIMUHeadingDegrees;
    }

    double targetIMUHeadingDegrees = currentIMUHeadingDegrees + deltaIMUHeadingDegrees;

    DrivetrainRC command = new DrivetrainRC(dtController, new PathChain(true,
            new Spin(targetIMUHeadingDegrees, rotationDirection))
    );

    rcController.qRC(command);

    Msg.log(getClass().getSimpleName(), "qSpinTurnToNearest45", "Queued spin turn to nearest 45 degrees, target=" + targetIMUHeadingDegrees);
  }
//
//  @Override
//  public void processCommands() {
////    if (dtManualMovementUpdated) {
////      clearSystemRCList();
////
////      driveByRobotCartesianENU(dtManual_X, dtManual_Y, dtManual_cwSpin - dtManual_ccwSpin, currentIMUHeading);
////      setMotorPowers(targetMotorPowerSettings);
////      dtManualMovementUpdated = false;
////    }
//
//
////    else if (!isSystemRCListEmpty()) {
//////      Msg.log("MecanumDrivetrain", "processCommands", "here3");
////      if (systemRCList.get(0).processRC()) {
////
////        systemRCList.remove(0);
////      }
////
////      setMotorPowers(targetMotorPowerSettings);
////    }
//  }
}
