package org.hexnibble.corelib.robot.drivetrain;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.motion.DriveController;
import org.hexnibble.corelib.robot_system.NewCoreRobotSystem;

abstract public class BaseDrivetrain extends NewCoreRobotSystem {
   DriveController dtController;

   protected double dtManual_X;  // X Joystick movement, range -1.0 (left) to +1.0 (right)
//   protected double previousDTManual_X;
   protected double dtAuto_X;    // X range -1.0 (left) to +1.0 (right)

   protected double dtManual_Y;  // Y Joystick movement, range -1.0 (backward) to +1.0 (forward)
//   protected double previousDTManual_Y;
   protected double dtAuto_Y;    // Y range -1.0 (left) to +1.0 (right)

   protected double dtManual_cwSpin; // Spin, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
//   protected double previousDTManual_cwSpin;
   protected double dtManual_ccwSpin; // Spin, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
//   protected double previousDTManual_ccwSpin;
   protected double dtAuto_Spin;    // Spin, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.

   protected boolean dtManualMovementUpdated;

   protected double currentIMUHeading;

//   protected PIDSettings rotationPIDSettings =
//         new PIDSettings(
//               ConfigFile.DRIVETRAIN_ROTATION_PID_Ks,
//               ConfigFile.DRIVETRAIN_ROTATION_PID_Kp,
//               ConfigFile.DRIVETRAIN_ROTATION_PID_Ki,
//               ConfigFile.DRIVETRAIN_ROTATION_PID_Kd);

//   // Create PID Controllers
//   protected dtRotationPIDController rotationPIDController = new dtRotationPIDController(
//         rotationPIDSettings, Math.toRadians(2), dtRotationPIDController.ROTATION_DIRECTION.SHORTEST );

   // Minimum threshold for motor power to exceed to send a new motor command (compared to the
   // previously sent value)
   protected final double MOTOR_POWER_THRESHOLD_FOR_NEW_COMMAND = 0.001;

   public BaseDrivetrain(@NonNull HardwareMap hwMap) {
      super(hwMap, "Drivetrain");
      dtController = new DriveController(this);
   }

   @Override
   public void resetSystem() {
      super.resetSystem();

      dtManualMovementUpdated = false;
      dtManual_X = 0.0;
      dtManual_Y = 0.0;
      dtManual_cwSpin = 0.0;
      dtManual_ccwSpin = 0.0;

//      previousDTManual_X = 0.0;
//      previousDTManual_Y = 0.0;
//      previousDTManual_cwSpin = 0.0;
//      previousDTManual_ccwSpin = 0.0;

      dtAuto_X = 0.0;
      dtAuto_Y = 0.0;
      dtAuto_Spin = 0.0;

      currentIMUHeading = 0.0;

      dtController = new DriveController(this);
   }


   public double getDtManualMovementX() {
      return dtManual_X;
   }

   /**
    * Set the X value for a manual drivetrain movement.
    *
    * @param X Movement in X direction, range -1.0 (left) to +1.0 (right)
    */
   public void setDrivetrainManualMovement_X(double X) {
      // Only indicate a new value if:
      // 1) Requested value is zero and the previous value is non-zero, OR
      // 2) Requested value is different from previous value by greater than threshold
      if (((X == 0.0) && (dtManual_X != 0.0))
         || (Math.abs(X - dtManual_X) > 0.01)) {
//         previousDTManual_X = dtManual_X;
         dtManual_X = X;
         dtManualMovementUpdated = true;
      }
   }

   public double getDtAutoMovementX() {
      return dtAuto_X;
   }

   public void setDtAutoMovementX(double x) {
      dtAuto_X = x;
   }

   public double getDtManualMovementY() {
      return dtManual_Y;
   }

   /**
    * Set the Y value for a manual drivetrain movement.
    *
    * @param Y Movement in Y direction, range -1.0 (backward) to +1.0 (forward). Remember Y values
    *     need to be flipped when taken directly from the joystick.
    */
   public void setDrivetrainManualMovement_Y(double Y) {
      // Only indicate a new value if:
      // 1) Requested value is zero and the previous value is non-zero, OR
      // 2) Requested value is different from previous value by greater than threshold
      if (((Y == 0.0) && (dtManual_Y != 0.0))
            || (Math.abs(Y - dtManual_Y) > 0.01)) {
//         previousDTManual_Y = dtManual_Y;
         dtManual_Y = Y;
         dtManualMovementUpdated = true;
      }
   }

   public double getDtAutoMovementY() {
      return dtAuto_Y;
   }

   public void setDtAutoMovementY(double y) {
      dtAuto_Y = y;
   }

   /**
    * Sets the clockwise spin for drivetrain movement.
    *
    * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
    */
   public void setDrivetrainManualMovement_cwSpin(double spin) {
      // Only indicate a new value if:
      // 1) Requested value is zero and the previous value is non-zero, OR
      // 2) Requested value is different from previous value by greater than threshold
      if (((spin == 0.0) && (dtManual_cwSpin != 0.0))
            || (Math.abs(spin - dtManual_cwSpin) > 0.01)) {
//         previousDTManual_cwSpin = dtManual_cwSpin;
         dtManual_cwSpin = spin;
         dtManualMovementUpdated = true;
      }
   }

   /**
    * Sets the counter-clockwise spin for drivetrain movement.
    *
    * @param spin CCW as a positive number from 0 - 1.0
    */
   public void setDrivetrainManualMovement_ccwSpin(double spin) {
      // Only indicate a new value if:
      // 1) Requested value is zero and the previous value is non-zero, OR
      // 2) Requested value is different from previous value by greater than threshold
      if (((spin == 0.0) && (dtManual_ccwSpin != 0.0))
            || (Math.abs(spin - dtManual_ccwSpin) > 0.01)) {
//         previousDTManual_ccwSpin = dtManual_ccwSpin;
         dtManual_ccwSpin = spin;
         dtManualMovementUpdated = true;
      }
   }

   public void setDtAutoMovementSpin(double spin) {
      dtAuto_Spin = spin;
   }

   public double getDtManualMovementSpin() {
      return dtManual_cwSpin - dtManual_ccwSpin;
   }

   public double getDtAutoMovementSpin() {
      return dtAuto_Spin;
   }

   public boolean isManualMovementUpdated() {
      return dtManualMovementUpdated;
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
    *     to field's zero heading (degrees)
    */
   abstract public void driveByRobotCartesianENU(double X, double Y, double spin,
                                                 double offsetToConvertToFieldCentricHeadingDegrees);

   /**
    * Move mecanum drivetrain based on robot-centric compass heading
    *
    * @param headingDegrees Robot-centric compass heading (0 - 360 degrees), where 0 deg is forward
    * @param motorPower Motor power for translation (must be a positive value from 0.0 - 1.0)
    * @param spin Spin speed, range -1.0 (CW) to +1.0 (CCW), using right-hand rule.
    */
//   abstract public void driveByRobotCompassHeading(double headingDegrees,
//                                                   float motorPower, double spin);

   /** Brake drivetrain by setting all motors to 0 power. */
   abstract public void brakeDrivetrain();

   abstract public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode);

   public void setCurrentIMUHeading(double currentIMUHeading) {
      this.currentIMUHeading = currentIMUHeading;
   }

   public DriveController getDtController() {
      return dtController;
   }
}
