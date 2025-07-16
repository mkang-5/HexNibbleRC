package org.hexnibble.corelib.wrappers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearMotor extends WheelMotor {
  private final double minPosition_mm, maxPosition_mm;

  /**
   * Constructor for LinearMotor class. Use for CASCADE rigging.
   *
   * @param hwMap Hardware map
   * @param motorName Name of the motor as specified in the driver station configuration
   * @param motorModel Motor model
   * @param runDirection Motor run direction
   * @param runMode Motor run mode
   * @param encoderType The type of encoder attached (INTERNAL, REV, GO_BILDA) to the encoderType
   *     port associated with this motor's power port.
   * @param encoderDirection Only used for external encoders (FORWARD, REVERSE)
   * @param numberOfSlides The number of slides on the lift (exclude base stage)
   * @param externalGearReduction External gear reduction. This value is ignored if the Rev
   *     encoderType use is set to true
   * @param outputDiameter_mm Diameter of the output (e.g. wheel, gear, etc) attached to this motor.
   * @param minPosition_mm Minimum position (mm) of the linear mechanism
   * @param maxPosition_mm Maximum position (mm) of the linear mechanism
   * @param targetPositionTolerance_mm Target position tolerance (in mm)
   */
  public LinearMotor(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotorSimple.Direction encoderDirection,
      int numberOfSlides,
      double externalGearReduction,
      double outputDiameter_mm,
      double minPosition_mm,
      double maxPosition_mm,
      int targetPositionTolerance_mm) {
    super(
        hwMap,
        motorName,
        motorModel,
        runDirection,
        runMode,
        encoderType,
        encoderDirection,
        externalGearReduction,
        outputDiameter_mm);

    this.minPosition_mm = minPosition_mm;
    this.maxPosition_mm = maxPosition_mm;

    if (numberOfSlides < 1) throw new AssertionError();
    countsPer_mm *= numberOfSlides;

    motor.setTargetPositionTolerance((int) (countsPer_mm * targetPositionTolerance_mm));
  }

  /**
   * Constructor for LinearMotor class. Use for CONTINUOUS rigging
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
   * @param outputDiameter_mm Diameter of the output (e.g. wheel, gear, etc) attached to this motor.
   * @param minPosition_mm Minimum position (mm) of the linear mechanism
   * @param maxPosition_mm Maximum position (mm) of the linear mechanism
   * @param targetPositionTolerance_mm Target position tolerance (in mm)
   */
  public LinearMotor(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotorSimple.Direction encoderDirection,
      double externalGearReduction,
      double outputDiameter_mm,
      double minPosition_mm,
      double maxPosition_mm,
      int targetPositionTolerance_mm) {
    super(
        hwMap,
        motorName,
        motorModel,
        runDirection,
        runMode,
        encoderType,
        encoderDirection,
        externalGearReduction,
        outputDiameter_mm);

    this.minPosition_mm = minPosition_mm;
    this.maxPosition_mm = maxPosition_mm;

    motor.setTargetPositionTolerance((int) (countsPer_mm * targetPositionTolerance_mm));
  }

  /**
   * Rotate the motor to a specified distance (based on wheel/sprocket diameter) RELATIVE TO encoder
   * count of zero using specified power. This is used for linear type of movements (e.g. using
   * chain).
   *
   * @param position_mm Target position (mm)
   * @param motorPower Motor power for this movement (0 - 1)
   * @param enforceLimits Enforce range limits on the movement. This can be set false if the encoder
   *     become out of sync (e.g. if a belt skips)
   * @return New target position (mm), which may be different from the requested target if enforcing
   *     limits and it was out of the range.
   */
  public double moveToPosition_mm(double position_mm, double motorPower, boolean enforceLimits) {
    // Check limits
    if (enforceLimits) {
      if (position_mm < minPosition_mm) position_mm = minPosition_mm;
      else if (position_mm > maxPosition_mm) position_mm = maxPosition_mm;
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
}
