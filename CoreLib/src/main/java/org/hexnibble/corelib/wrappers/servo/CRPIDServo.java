package org.hexnibble.corelib.wrappers.servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.motion.pid.dtRotationPIDController;

public class CRPIDServo extends CRServo {
//   private double currentServoPositionDegrees;

   private final dtRotationPIDController pid;

   private double servoTargetAngleDegrees;

   public CRPIDServo(HardwareMap hwMap, String servoName, SERVO_MODEL servoModel,
                     String encoderName, DcMotorSimple.Direction encoderDirection) {
      super(hwMap, servoName, servoModel, -1.0, 1.0, encoderName, encoderDirection);

      pid = new dtRotationPIDController(0.0, ConfigFile.SWERVE_PID_Kp,
            ConfigFile.SWERVE_PID_Ki,ConfigFile.SWERVE_PID_Kd, 0.01,
            dtRotationPIDController.ROTATION_DIRECTION.SHORTEST);

      servoTargetAngleDegrees = 0.0;
   }

   public void setTargetAngleDegrees(double targetAngleDegrees) {
      servoTargetAngleDegrees = targetAngleDegrees;
   }

   public void processCommand() {
      double currentAngleDegrees = readServoPositionFromEncoderDegrees();
      double errorDegrees = currentAngleDegrees - servoTargetAngleDegrees;
      double power;

      if (Math.abs(errorDegrees) > 0.5) {
         power = pid.calculateNewControlValue(Math.toRadians(errorDegrees));
      }
      else {
         // If there is minimal error, turn off the power
         power = 0.0;
         pid.resetTargetCounter();
         pid.resetErrorHistories();
      }

      setServoSpeed(power);
   }
}
