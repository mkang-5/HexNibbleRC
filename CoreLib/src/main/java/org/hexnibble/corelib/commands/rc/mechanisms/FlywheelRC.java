package org.hexnibble.corelib.commands.rc.mechanisms;

import java.util.function.Supplier;

import org.hexnibble.corelib.commands.rc.RC;
import org.hexnibble.corelib.robot_system.FlyWheelSystem;

public class FlywheelRC extends RC {
  private final FlyWheelSystem flyWheelSystem;
//  private final WheelMotor motor;

//  public enum ROTATION_DIRECTION {
//    CLOCKWISE,
//    COUNTERCLOCKWISE
//  }

//  private final ROTATION_DIRECTION rotationDirection;

//  PIDSettings flywheelPIDSettings =
//        new PIDSettings(
//              Constants.FLYWHEEL_PID_Ks,
//              Constants.FLYWHEEL_PID_Kp,
//              Constants.FLYWHEEL_PID_Ki,
//              Constants.FLYWHEEL_PID_Kd);
//  private final PIDController flywheelPIDController;

  private final int targetRPM;

//  private int currentEncoderCount;

  private final Supplier<Long> deltaTimeSupplier;

//  private double controlValue;

//  public FlywheelRC(WheelMotor wheelMotor, ROTATION_DIRECTION rotationDirection,
//                    Supplier<Integer> deltaTimeSupplier, int targetRPM) {
  public FlywheelRC(FlyWheelSystem flyWheelSystem, // ROTATION_DIRECTION rotationDirection,
      Supplier<Long> deltaTimeSupplier, int targetRPM) {

    this.flyWheelSystem = flyWheelSystem;
//    this.motor = wheelMotor;
//    this.rotationDirection = rotationDirection;
    this.deltaTimeSupplier = deltaTimeSupplier;
    this.targetRPM = targetRPM;

//    // Create PID Controller
//    double targetToleranceRPM = 5.0;
//    flywheelPIDController = new PIDController(flywheelPIDSettings, targetToleranceRPM);
  }

  @Override
  protected void onStartCommand() {
    flyWheelSystem.setTargetVelocityRPM(targetRPM);
    flyWheelSystem.start();
  }

  @Override
  protected void onCompleteCommand() {
    flyWheelSystem.setTargetVelocityRPM(0);
  }

  @Override
  protected void processCommand() {
    long deltaTimeBetweenReadings = deltaTimeSupplier.get();

    if (deltaTimeBetweenReadings != 0) {
      if (flyWheelSystem.getTargetVelocityRPM() == 0) {
        setCommandComplete();
      }
      else {
        flyWheelSystem.updateSystem(deltaTimeBetweenReadings);
      }

//      int previousEncoderCount = currentEncoderCount;
//      currentEncoderCount = motor.getCurrentPosition();
//
//      double countsPer_ms =
//          (double) (currentEncoderCount - previousEncoderCount) / deltaTimeBetweenReadings;
//      double currentRPM = countsPer_ms / motor.getEffectiveCountsPerRev() * 1000.0 * 60.0;
//      controlValue = flywheelPIDController.calculateNewControlValue(targetRPM - currentRPM);
//      motor.setPower(controlValue);
    }
  }
}
