package org.hexnibble.corelib.commands.rc.mechanisms;

import java.util.function.Supplier;
import org.hexnibble.corelib.commands.rc.RC;
import org.hexnibble.corelib.motion.pid.PIDController;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

@Deprecated(since = "5/24/25", forRemoval = true)
public class FlywheelRCJ extends RC {
  private final WheelMotor motor;

  public enum ROTATION_DIRECTION {
    CLOCKWISE,
    COUNTERCLOCKWISE
  }

  private final ROTATION_DIRECTION rotationDirection;

  private final PIDController PIDController;

  private final int targetRPM;

  private int currentEncoderCount;

  private final Supplier<Integer> deltaTimeSupplier;

  private double controlValue;

  public FlywheelRCJ(
      WheelMotor wheelMotor,
      ROTATION_DIRECTION rotationDirection,
      PIDSettings PIDSettings,
      Supplier<Integer> deltaTimeSupplier,
      int targetRPM) {
    this.motor = wheelMotor;
    this.rotationDirection = rotationDirection;
    this.deltaTimeSupplier = deltaTimeSupplier;
    this.targetRPM = targetRPM;

    // Create PID Controller
    double targetToleranceRPM = 5.0;
    PIDController = new PIDController(PIDSettings, targetToleranceRPM);
  }

  @Override
  protected void processCommand() {
    int deltaTimeBetweenReadings = deltaTimeSupplier.get();

    if (deltaTimeBetweenReadings != 0) {
      int previousEncoderCount = currentEncoderCount;
      currentEncoderCount = motor.getCurrentPosition();

      double countsPer_ms =
          (double) (currentEncoderCount - previousEncoderCount) / deltaTimeBetweenReadings;
      double currentRPM = countsPer_ms / motor.getEffectiveCountsPerRev() * 1000.0 * 60.0;
      controlValue = PIDController.calculateNewControlValue(targetRPM - currentRPM);
      motor.setPower(controlValue);
    }
  }
}
