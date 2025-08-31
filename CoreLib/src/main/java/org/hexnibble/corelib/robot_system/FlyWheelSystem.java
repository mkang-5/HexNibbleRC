package org.hexnibble.corelib.robot_system;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.motion.pid.PIDController;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public class FlyWheelSystem extends NewCoreRobotSystem {
   private int previousEncoder1Count;
   private int previousEncoder2Count;
   private int targetVelocityRPM;
   private final WheelMotor wheelMotor1;
   private final WheelMotor wheelMotor2;
   private final double CPR_to_RPM;

   PIDSettings flywheelPIDSettings =
         new PIDSettings(
               Constants.FLYWHEEL_PID_Ks,
               Constants.FLYWHEEL_PID_Kp,
               Constants.FLYWHEEL_PID_Ki,
               Constants.FLYWHEEL_PID_Kd);

   protected PIDController flywheelPIDController1;
   protected PIDController flywheelPIDController2;

   public FlyWheelSystem(@NonNull HardwareMap hwMap, String systemName,
                         WheelMotor wheelMotor1, WheelMotor wheelMotor2) {
      super(hwMap, systemName);

      this.wheelMotor1 = wheelMotor1;
      this.wheelMotor2 = wheelMotor2;

      flywheelPIDController1 = new PIDController(flywheelPIDSettings, 5);

      if (wheelMotor2 != null) {
         flywheelPIDController2 = new PIDController(flywheelPIDSettings, 5);
      }

      CPR_to_RPM = 1000.0 * 60.0 / wheelMotor1.getEffectiveCountsPerRev();
   }

   @Override
   public void resetSystem() {
      super.resetSystem();

      wheelMotor1.reset();
      if (wheelMotor2 != null) {
         wheelMotor2.reset();
      }
   }

   public void start() {
      previousEncoder1Count = wheelMotor1.getCurrentPosition();
      if (wheelMotor2 != null) {
         previousEncoder2Count = wheelMotor2.getCurrentPosition();
      }
   }

   public int getTargetVelocityRPM() {
      return targetVelocityRPM;
   }

   public void setTargetVelocityRPM(int targetVelocityRPM) {
      Msg.log(getClass().getSimpleName(), "setTargetVelocityRPM", "Setting target velocity to " + targetVelocityRPM);
      this.targetVelocityRPM = targetVelocityRPM;

      if (targetVelocityRPM == 0) {
         wheelMotor1.setPower(0.0);

         if (wheelMotor2 != null) {
            wheelMotor2.setPower(0.0);
         }
      }
   }

   public void updateSystem(long deltaTimeBetweenReadings_ms) {
      int currentEncoderCount = wheelMotor1.getCurrentPosition();

      double countsPer_ms = (currentEncoderCount - previousEncoder1Count) / (double) deltaTimeBetweenReadings_ms;
      double currentRPM = countsPer_ms * CPR_to_RPM;

      double error = targetVelocityRPM - currentRPM;
      double controlValue = flywheelPIDController1.calculateNewControlValue(error);

      previousEncoder1Count = currentEncoderCount;
      wheelMotor1.setPower(controlValue);

      Msg.log(getClass().getSimpleName(), "updateSystem", "TargetRPM=" + targetVelocityRPM
            + ", M1: currentEncoder=" + currentEncoderCount
//            + ", TimeBetwReadings=" + deltaTimeBetweenReadings_ms
//            + ", countsPer_ms=" + countsPer_ms
            + ", currentRPM=" + currentRPM
            + ", Motor1 Error=" + error
            + ", controlValue=" + controlValue);

      if (wheelMotor2 != null) {
         int currentEncoder2Count = wheelMotor2.getCurrentPosition();

         double error2 = targetVelocityRPM
               - ((double) (currentEncoder2Count - previousEncoder2Count) / deltaTimeBetweenReadings_ms) * CPR_to_RPM;

         previousEncoder2Count = currentEncoder2Count;
         wheelMotor2.setPower(flywheelPIDController2.calculateNewControlValue(error2));

         Msg.log(getClass().getSimpleName(), "updateSystem", "Motor2 Error=" + error2);
      }
   }
}
