package org.hexnibble.corelib.wrappers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelMotor extends BaseMotorWrapper {
  protected final double wheelDiameter_mm;
  protected double countsPer_mm;

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
   * @param wheelDiameter_mm Diameter of the wheel attached to this motor. If the encoderType port
   *     is being used for monitoring a separate wheel, then specify the diameter of the wheel
   *     attached to the encoderType.
   */
  public WheelMotor(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotorSimple.Direction encoderDirection,
      double externalGearReduction,
      double wheelDiameter_mm) {
    super(
        hwMap,
        motorName,
        motorModel,
        runDirection,
        runMode,
        encoderType,
        encoderDirection,
        externalGearReduction);

    this.wheelDiameter_mm = wheelDiameter_mm;

    if (encoderType == ENCODER.INTERNAL) {
      countsPer_mm =
          countsPerRev
              * internalGearReduction
              * this.externalGearReduction
              / (this.wheelDiameter_mm * Math.PI);
    } else {
      countsPer_mm = countsPerRev / (this.wheelDiameter_mm * Math.PI);
    }
  }

  /**
   * Constructor specifying target position tolerance
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
   * @param wheelDiameter_mm Diameter of the wheel attached to this motor. If the encoderType port
   *     is being used for monitoring a separate wheel, then specify the diameter of the wheel
   *     attached to the encoderType.
   * @param targetPositionTolerance Target position tolerance (in counts)
   */
  public WheelMotor(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotorSimple.Direction encoderDirection,
      double externalGearReduction,
      double wheelDiameter_mm,
      int targetPositionTolerance) {
    super(
        hwMap,
        motorName,
        motorModel,
        runDirection,
        runMode,
        encoderType,
        encoderDirection,
        externalGearReduction,
        targetPositionTolerance);

    this.wheelDiameter_mm = wheelDiameter_mm;

    if (encoderType == ENCODER.INTERNAL) {
      countsPer_mm =
          countsPerRev
              * internalGearReduction
              * this.externalGearReduction
              / (this.wheelDiameter_mm * Math.PI);
    } else {
      countsPer_mm = countsPerRev / (this.wheelDiameter_mm * Math.PI);
    }
  }

  /** POSITION FUNCTIONS */
  public double getCurrentPosition_mm() {
    return getCurrentPosition() / countsPer_mm;
  }

  public double getTargetPosition_mm() {
    return ((double) getTargetPosition()) / countsPer_mm;
  }

  public void setTargetPosition_mm(double targetPosition_mm) {
    int targetPositionCounts =
        (int) (targetPosition_mm * countsPer_mm); // Convert target distance to encoder counts
    setTargetPosition(targetPositionCounts);
  }
}
