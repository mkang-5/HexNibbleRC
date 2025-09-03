package org.hexnibble.corelib.robot.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.servo.BaseServoWrapper;
import org.hexnibble.corelib.wrappers.servo.CRPIDServo;

public class SwerveModule extends WheelModule {
   CRPIDServo servo;

   public SwerveModule(HardwareMap hwMap, String motorName,
                       BaseMotorWrapper.MOTOR_MODEL motorModel,
                       DcMotor.Direction motorRunDirection, DcMotor.RunMode runMode,
                       BaseMotorWrapper.ENCODER encoderType,
                       DcMotor.Direction motorEncoderDirection,
                       double extGearReduction, double wheelDiameterMM,
                       int targetPositionTolerance,
                       String servoName, BaseServoWrapper.SERVO_MODEL servoModel,
                       String servoEncoderName, DcMotorSimple.Direction servoEncoderDirection) {

      super(hwMap, motorName, motorModel, motorRunDirection, runMode,
            encoderType, motorEncoderDirection, extGearReduction, wheelDiameterMM,
            targetPositionTolerance);

      servo = new CRPIDServo(hwMap, servoName, servoModel, servoEncoderName, servoEncoderDirection);
   }

   public void targetAngleDegrees(double targetAngleDegrees) {
      servo.setTargetAngleDegrees(targetAngleDegrees);
   }

   public void processCommand() {
      servo.processCommand();
   }
}
