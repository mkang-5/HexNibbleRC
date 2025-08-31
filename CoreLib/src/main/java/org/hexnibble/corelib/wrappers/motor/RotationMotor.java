package org.hexnibble.corelib.wrappers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotationMotor extends BaseMotorWrapper {
  private final double minPositionDegrees, maxPositionDegrees, countsPerDegree;
  private double currentPositionDegrees;

  /**
   * Constructor without target position tolerance
   *
   * @param hwMap Hardware map
   * @param motorName Name of the motor as specified in the driver station configuration
   * @param motorModel Motor model
   * @param runDirection Motor run direction
   * @param runMode Motor run mode
   * @param encoderType The type of encoder attached (INTERNAL, REV, GO_BILDA) to the encoderType
   *     port associated with this motor's power port.
   * @param encoderDirection Only used for external encoders (FORWARD, REVERSE)
   * @param externalGearReduction External gear reduction. This value is ignored if the Rev
   *     encoderType use is set to true
   * @param minPositionDegrees Minimum position (mm) of the linear mechanism
   * @param maxPositionDegrees Maximum position (mm) of the linear mechanism
   * @param targetPositionToleranceDegrees Target position tolerance (in degrees)
   */
  public RotationMotor(HardwareMap hwMap, String motorName, MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection, DcMotor.RunMode runMode,
      ENCODER encoderType, DcMotorSimple.Direction encoderDirection,
      double externalGearReduction,
      double minPositionDegrees, double maxPositionDegrees, double targetPositionToleranceDegrees) {

    super(hwMap, motorName, motorModel, runDirection, runMode,
        encoderType, encoderDirection, externalGearReduction);

    countsPerDegree = countsPerRev * internalGearReduction * this.externalGearReduction / 360.0;
    currentPositionDegrees = 0.0;

    this.minPositionDegrees = minPositionDegrees;
    this.maxPositionDegrees = maxPositionDegrees;

    motor.setTargetPositionTolerance((int) (countsPerDegree * targetPositionToleranceDegrees));
  }

  /**
   * Rotate motor to the specified angle (in degrees) RELATIVE TO encoder count of zero using
   * specified power.
   *
   * @param targetAngleDegrees Target angle in degrees
   * @param motorPower Motor power for this movement (0 - 1)
   * @param enforceLimits Enforce range limits on the movement. This can be set false if the encoder
   *     become out of sync (e.g. if a belt skips)
   * @return New target position (degrees), which may be different from the requested target if
   *     enforcing limits and it was out of the range.
   */
  public double moveToAngle(double targetAngleDegrees, double motorPower, boolean enforceLimits) {
    // Check limits
    if (enforceLimits) {
      if (targetAngleDegrees < minPositionDegrees) {
        targetAngleDegrees = minPositionDegrees;
      } else if (targetAngleDegrees > maxPositionDegrees) {
        targetAngleDegrees = maxPositionDegrees;
      }
    }

    int target =
        (int) (targetAngleDegrees * countsPerDegree); // Convert target angle to encoder counts

    setTargetPosition(target);
    setPower(Math.abs(motorPower));

    currentPositionDegrees = targetAngleDegrees;
    return targetAngleDegrees;
  }

  /**
   * Rotate motor a specified angle (degrees) from the current position
   *
   * @param deltaAngleDegrees Angle to rotate (degrees)
   * @param motorPower Motor power
   */
  public void rotateDeltaAngle(double deltaAngleDegrees, double motorPower) {
    moveToAngle((deltaAngleDegrees + currentPositionDegrees), motorPower, true);
  }

  public double getCurrentPositionDegrees() {
    return getCurrentPosition() / countsPerDegree;
  }

  public double getTargetPositionDegrees() {
    return ((double) getTargetPosition()) / countsPerDegree;
  }

  /**
   * Move to the minimum angle.
   *
   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
   * @return New target position (degrees)
   */
  public double moveToMinAngle(double motorPower) {
    moveToAngle(minPositionDegrees, motorPower, false);
    return minPositionDegrees;
  }

  /**
   * Move to the max angle.
   *
   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
   * @return New target position (degrees)
   */
  public double moveToMaxAngle(double motorPower) {
    moveToAngle(maxPositionDegrees, motorPower, false);
    return maxPositionDegrees;
  }

  /**
   * Stop the motor at the current position and save the position. This is used in RUN_TO_POSITION
   * mode. If power is set to zero, the motor may move based on gravity depending on its weight and
   * gearing. If power is left as is, then the motor will remain running to try and maintain the
   * position.
   *
   * @param setPowerToZero If true, will set the Power to 0.
   */
  public void stopMotor(boolean setPowerToZero) {
    int currentPositionCounts = getCurrentPosition();
    setTargetPosition(currentPositionCounts);
    currentPositionDegrees = currentPositionCounts / countsPerDegree;

    if (setPowerToZero) {
      motor.setPower(0.0);
    }
  }
}
