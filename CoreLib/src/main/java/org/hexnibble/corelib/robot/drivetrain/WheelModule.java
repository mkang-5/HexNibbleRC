package org.hexnibble.corelib.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

/**
 * A base class for a generic wheel module for a drivetrain.
 */
public class WheelModule {
   protected final WheelMotor motor;

   public WheelModule(HardwareMap hwMap, String motorName,
                      BaseMotorWrapper.MOTOR_MODEL motorModel,
                      DcMotor.Direction motorRunDirection, DcMotor.RunMode runMode,
                      BaseMotorWrapper.ENCODER encoderType,
                      DcMotor.Direction encoderDirection,
                      double extGearReduction, double wheelDiameterMM,
                      int targetPositionTolerance) {

      motor = new WheelMotor(hwMap, motorName, motorModel, motorRunDirection, runMode,
            encoderType, encoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);
   }

   public void reset() {
      motor.reset();
   }

   public WheelMotor getWheelMotorObject() {
      return motor;
   }

   public void setMotorPower(double power) {
      motor.setPower(power);
   }

   public void setMotorPowerNoClamping(double power) {
      motor.setPowerNoClamping(power);
   }

   public double getMotorPower() {
      return motor.getMotorPower();
   }

   public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
      motor.setBrakeMode(brakeMode);
   }

   public double getMotorCurrent(CurrentUnit currentUnit) {
      return motor.getMotorCurrent(currentUnit);
   }

   public void setMotorRunMode(DcMotor.RunMode runMode) {
      motor.setRunMode(runMode);
   }
}