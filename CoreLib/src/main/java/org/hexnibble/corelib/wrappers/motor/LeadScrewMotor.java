package org.hexnibble.corelib.wrappers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LeadScrewMotor extends BaseMotorWrapper {
  private final double distancePerRotation_mm, minPosition_mm, maxPosition_mm, countsPer_mm;

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
   * @param distancePerRotation_mm Linear distance per rotation of the lead screw attached to this
   *     motor.
   * @param minPosition_mm Minimum position (mm) of the linear mechanism
   * @param maxPosition_mm Maximum position (mm) of the linear mechanism
   */
  public LeadScrewMotor(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotorSimple.Direction encoderDirection,
      double externalGearReduction,
      double distancePerRotation_mm,
      double minPosition_mm,
      double maxPosition_mm) {
    super(
        hwMap,
        motorName,
        motorModel,
        runDirection,
        runMode,
        encoderType,
        encoderDirection,
        externalGearReduction);

    this.distancePerRotation_mm = distancePerRotation_mm;
    this.minPosition_mm = minPosition_mm;
    this.maxPosition_mm = maxPosition_mm;

    if (encoderType == ENCODER.INTERNAL) {
      countsPer_mm =
          countsPerRev
              * internalGearReduction
              * this.externalGearReduction
              / this.distancePerRotation_mm;
    } else {
      countsPer_mm = countsPerRev / this.distancePerRotation_mm;
    }
  }

  public double getCurrentPosition_mm() {
    return getCurrentPosition() / countsPer_mm;
  }

  /**
   * Rotate the motor to a specified distance (based on lead distance/distance per rotation)
   * RELATIVE TO encoder count of zero using specified power.
   *
   * @param position_mm Target position (mm)
   * @param motorPower Motor power for this movement (0 - 1)
   * @param enforceLimits Enforce range limits on the movement.
   * @return New target position (mm), which may be different from the requested target if enforcing
   *     limits and it was out of the range.
   */
  public double moveToPosition_mm(double position_mm, double motorPower, boolean enforceLimits) {
    // Check limits
    if (enforceLimits) {
      if (position_mm < minPosition_mm) {
        position_mm = minPosition_mm;
      } else if (position_mm > maxPosition_mm) {
        position_mm = maxPosition_mm;
      }
    }

    int target = (int) (position_mm * countsPer_mm); // Convert target distance to encoder counts

    setTargetPosition(target);
    setPower(Math.abs(motorPower));

    return position_mm;
  }

  /**
   * Move toward the minimum distance (retraction).
   *
   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
   * @return New target position (mm)
   */
  public double moveToMinPosition(double motorPower) {
    moveToPosition_mm(minPosition_mm, motorPower, false);
    return minPosition_mm;
  }

  /**
   * Move to the max distance (extension).
   *
   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
   * @return New target position (mm)
   */
  public double moveToMaxPosition(double motorPower) {
    moveToPosition_mm(maxPosition_mm, motorPower, false);
    return maxPosition_mm;
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
    setTargetPosition(getCurrentPosition());

    if (setPowerToZero) {
      motor.setPower(0.0);
    }
  }
}
