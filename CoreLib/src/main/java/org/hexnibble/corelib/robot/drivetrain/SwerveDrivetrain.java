package org.hexnibble.corelib.robot.drivetrain;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.PolarCoords;
import org.hexnibble.corelib.motion.MotorPowerSettings;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.servo.BaseServoWrapper;

public class SwerveDrivetrain extends BaseDrivetrain {
   // Variables to hold motor objects.
   public enum WHEEL_MODULE_NAME {
      LF, RF, LB, RB
   }

   private final SwerveModule moduleLeftFront;
   private final SwerveModule moduleRightFront;
   private final SwerveModule moduleLeftBack;
   private final SwerveModule moduleRightBack;

   // Variables to hold target power for each motor
   private final MotorPowerSettings targetMotorPowerSettings = new MotorPowerSettings();

   // Minimum threshold for motor power to exceed to send a new motor command (compared to the
   // previously sent value)
   private final double MOTOR_POWER_THRESHOLD_FOR_NEW_COMMAND = 0.001;

   public SwerveDrivetrain(@NonNull HardwareMap hwMap,
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
                           double extGearReduction, int targetPositionTolerance,
                           String LFServoName, String LFServoEncoderName,
                           DcMotorSimple.Direction LFServoEncoderDirection,
                           String RFServoName, String RFServoEncoderName,
                           DcMotorSimple.Direction RFServoEncoderDirection,
                           String LBServoName, String LBServoEncoderName,
                           DcMotorSimple.Direction LBServoEncoderDirection,
                           String RBServoName, String RBServoEncoderName,
                           DcMotorSimple.Direction RBServoEncoderDirection,
                           BaseServoWrapper.SERVO_MODEL servoModel,
                           double wheelDiameterMM) {
      super(hwMap);

      moduleLeftFront =
            new SwerveModule(hwMap, LFMotorName, motorModel, LFMotorRunDirection, runMode,
                  LFEncoderType, LFEncoderDirection, extGearReduction, wheelDiameterMM,
                  targetPositionTolerance, LFServoName, servoModel,
                  LFServoEncoderName, LFServoEncoderDirection);
      moduleRightFront =
            new SwerveModule(hwMap, RFMotorName, motorModel, RFMotorRunDirection, runMode,
                  RFEncoderType, RFEncoderDirection, extGearReduction, wheelDiameterMM,
                  targetPositionTolerance, RFServoName, servoModel,
                  RFServoEncoderName, RFServoEncoderDirection);
      moduleLeftBack =
            new SwerveModule(hwMap, LBMotorName, motorModel, LBMotorRunDirection, runMode,
                  LBEncoderType, LBEncoderDirection, extGearReduction, wheelDiameterMM,
                  targetPositionTolerance, LBServoName, servoModel,
                  LBServoEncoderName, LBServoEncoderDirection);
      moduleRightBack =
            new SwerveModule(hwMap, RBMotorName, motorModel, RBMotorRunDirection, runMode,
                  RBEncoderType, RBEncoderDirection, extGearReduction, wheelDiameterMM,
                  targetPositionTolerance, RBServoName, servoModel,
                  RBServoEncoderName, RBServoEncoderDirection);
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
   public void driveByRobotCartesianENU(double X, double Y, double spin,
                                        double offsetToConvertToFieldCentricHeadingDegrees) {

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
         mPowers.LF = r;
         mPowers.RF = r;
         mPowers.LB = r;
         mPowers.RB = r;
      }
   }

//   /**
//    * Move mecanum drivetrain based on robot-centric compass heading
//    *
//    * @param headingDegrees Robot-centric compass heading (0 - 360 degrees), where 0 deg is forward
//    * @param motorPower Motor power for translation (must be a positive value from 0.0 - 1.0)
//    * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
//    */
//   @Override
//   public void driveByRobotCompassHeading(double headingDegrees,
//                                          float motorPower, double spin) {
//
//   }

   private void setMotorPowers(MotorPowerSettings targetMPowers) {
      moduleLeftFront.setMotorPowerNoClamping(targetMPowers.LF);
      moduleRightFront.setMotorPowerNoClamping(targetMPowers.RF);
      moduleLeftBack.setMotorPowerNoClamping(targetMPowers.LB);
      moduleRightBack.setMotorPowerNoClamping(targetMPowers.RB);
   }

   /** Brake drivetrain by setting all motors to 0 power. */
   @Override
   public void brakeDrivetrain() {
      moduleLeftFront.setMotorPower(0.0);
      moduleRightFront.setMotorPower(0.0);
      moduleLeftBack.setMotorPower(0.0);
      moduleRightBack.setMotorPower(0.0);
   };

   @Override
   public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
      moduleLeftFront.setBrakeMode(brakeMode);
      moduleRightFront.setBrakeMode(brakeMode);
      moduleLeftBack.setBrakeMode(brakeMode);
      moduleRightBack.setBrakeMode(brakeMode);
   }

   public void updateServoPositions() {

   }

   @Override
   public void processCommands() {
      if (dtManualMovementUpdated) {
         clearSystemRCList();

         driveByRobotCartesianENU(dtManual_X, dtManual_Y, dtManual_cwSpin - dtManual_ccwSpin, currentIMUHeading);

         updateServoPositions();
         setMotorPowers(targetMotorPowerSettings);
         dtManualMovementUpdated = false;
      }
   }
}
