package org.hexnibble.corelib.wrappers.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.hexnibble.corelib.misc.Constants;

public class BaseMotorWrapper {
  public enum MOTOR_MODEL {
    GoBildaYJ_30,
    GoBildaYJ_43,
    GoBildaYJ_60,
    GoBildaYJ_84,
    GoBildaYJ_117,
    GoBildaYJ_223,
    GoBildaYJ_312,
    GoBildaYJ_435,
    GoBildaYJ_1150,
    GoBildaYJ_1620,
    GoBildaYJ_6000,
    RevCoreHex,
    /**
     * @noinspection SpellCheckingInspection
     */
    RevUPHDHex
  }

  public static final double REV_GEAR_CARTRIDGE_5_1 = 5.23;
  public static final double REV_GEAR_CARTRIDGE_4_1 = 3.61;
  public static final double REV_GEAR_CARTRIDGE_3_1 = 2.89;

  public static final double AXON_MITER_GEAR = 52.0 / 18.0;

  protected final String motorName;
  protected final DcMotorImplEx motor;
  protected final DcMotor.Direction runDirection;
  protected final DcMotor.RunMode initialRunMode;
  protected DcMotor.RunMode currentRunMode;

  public enum ENCODER {
    INTERNAL,
    GO_BILDA_ODOPOD,
    REV
  }

  protected final ENCODER encoderType;
  protected final DcMotor.Direction encoderDirection;

  protected final double internalGearReduction;
  protected final double externalGearReduction;
  protected int countsPerRev;

  protected double power = 0.0;
  protected DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
  protected int targetPosition;
  protected int targetPositionToleranceCounts;

  protected double targetVelocityRPM;
  protected double targetVelocityToleranceRPM;

  /**
   * Constructor without target position tolerance
   *
   * @param hwMap Hardware map
   * @param motorName Name for this motor
   * @param motorModel Motor model
   * @param runDirection Run direction
   * @param runMode Run mode
   * @param encoderType The type of encoder attached (INTERNAL, REV, GO_BILDA) to the encoderType
   *     port associated with this motor's power port.
   * @param encoderDirection Only used for external encoders (FORWARD, REVERSE). It will be ignored
   *     if an INTERNAL encoderType is specified
   * @param externalGearReduction External gear reduction
   */
  public BaseMotorWrapper(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotor.Direction encoderDirection,
      double externalGearReduction) {
    if (hwMap == null) {
      throw new NullPointerException();
    }

    motor = hwMap.get(DcMotorImplEx.class, motorName); // Get the motor object
    this.motorName = motorName;

    motor.setPower(0.0);
    motor.setZeroPowerBehavior(zeroPowerBehavior);

    // Set run direction
    this.runDirection = runDirection;
    motor.setDirection(this.runDirection);
    if (encoderType == ENCODER.INTERNAL) {
      this.encoderDirection = runDirection;
    } else {
      this.encoderDirection = encoderDirection;
    }

    // Set motor run mode
    initialRunMode = runMode; // Store the current run mode
    setRunMode(initialRunMode); // Set the motor to the requested run mode

    this.externalGearReduction = externalGearReduction;

    switch (motorModel) {
      case GoBildaYJ_30 ->
          internalGearReduction =
              (1.0 + (46.0 / 17.0))
                  * (1.0 + (46.0 / 17.0))
                  * (1.0 + (46.0 / 17.0))
                  * (1.0 + (46.0 / 17.0));
      case GoBildaYJ_43 ->
          internalGearReduction =
              (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0));
      case GoBildaYJ_60 ->
          internalGearReduction =
              (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 17.0));
      case GoBildaYJ_84 ->
          internalGearReduction =
              (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 17.0));
      case GoBildaYJ_117 ->
          internalGearReduction =
              (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 17.0));
      case GoBildaYJ_223 -> internalGearReduction = (1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0));
      case GoBildaYJ_312 -> internalGearReduction = (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 11.0));
      case GoBildaYJ_435 -> internalGearReduction = (1.0 + (46.0 / 17.0)) * (1.0 + (46.0 / 17.0));
      case GoBildaYJ_1150 -> internalGearReduction = (1.0 + (46.0 / 11.0));
      case GoBildaYJ_1620 -> internalGearReduction = (1.0 + (46.0 / 17.0));
      case GoBildaYJ_6000, RevUPHDHex -> internalGearReduction = 1.0;
      case RevCoreHex -> internalGearReduction = 72.0;
      default ->
          throw new IllegalArgumentException(
              "Unknown motor model information in BaseMotorWrapper constructor");
    }

    this.encoderType = encoderType;
    switch (encoderType) {
      case INTERNAL -> {
        if (motorModel == MOTOR_MODEL.RevCoreHex) {
          countsPerRev = 4;
        } else {
          countsPerRev = 28; // GoBilda YJ motors are 28
        }
      }
      case REV -> countsPerRev = 8192;
      case GO_BILDA_ODOPOD -> countsPerRev = 2000;
    }

    targetPosition = motor.getTargetPosition();
    targetPositionToleranceCounts = motor.getTargetPositionTolerance();
  }

  /**
   * Constructor specifying target position tolerance
   *
   * @param hwMap Hardware map
   * @param motorName Name of the motor as specified in the driver station configuration
   * @param motorModel Motor model
   * @param runDirection Run direction
   * @param runMode Run mode
   * @param encoderType The type of encoder attached (INTERNAL, REV, GO_BILDA) to the encoderType
   *     port associated with this motor's power port.
   * @param encoderDirection Only used for external encoders (FORWARD, REVERSE). It will be ignored
   *     if an INTERNAL encoderType is specified
   * @param externalGearReduction External gear reduction
   * @param targetPositionToleranceCounts Target position tolerance (counts)
   */
  public BaseMotorWrapper(
      HardwareMap hwMap,
      String motorName,
      MOTOR_MODEL motorModel,
      DcMotor.Direction runDirection,
      DcMotor.RunMode runMode,
      ENCODER encoderType,
      DcMotor.Direction encoderDirection,
      double externalGearReduction,
      int targetPositionToleranceCounts) {
    this(
        hwMap,
        motorName,
        motorModel,
        runDirection,
        runMode,
        encoderType,
        encoderDirection,
        externalGearReduction);

    this.targetPositionToleranceCounts = targetPositionToleranceCounts;
    motor.setTargetPositionTolerance(this.targetPositionToleranceCounts);
  }

  /**
   * Call this function to reinitialize this motor when restarting an OpMode. It will revert back to
   * the most recent stored values. This function will NOT reset encoders.
   */
  public void reset() {

    motor.setZeroPowerBehavior(zeroPowerBehavior);
    motor.setTargetPositionTolerance(targetPositionToleranceCounts);
    motor.setDirection(runDirection);
    setRunMode(initialRunMode); // Set the motor to the requested run mode
  }

  public DcMotor.RunMode getRunMode() {
    return motor.getMode();
  }

  public void setBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
    motor.setZeroPowerBehavior(brakeMode);
    zeroPowerBehavior = brakeMode;
  }

  /**
   * Set the motor to the requested run mode. For RUN_TO_POSITION, the motor's target position will
   * first be set to its current position so that it doesn't start moving.
   *
   * @param runMode Desired run mode
   */
  public void setRunMode(DcMotor.RunMode runMode) {
    if (runMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    else if (runMode == DcMotor.RunMode.RUN_TO_POSITION) {
      // Reset the target position to the current position so the motor doesn't start running
      targetPosition = motor.getCurrentPosition();
      motor.setTargetPosition(targetPosition);
      motor.setMode(runMode);
    }
    else { // RUN_USING_ENCODER or RUN_WITHOUT_ENCODER
      motor.setMode(runMode);
    }
    currentRunMode = runMode;
  }

  /** Reset the associated encoder. */
  public void resetEncoder() {
    DcMotor.RunMode currentRunMode = motor.getMode(); // Store current run mode
    power = 0.0;
    setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    setRunMode(currentRunMode); // Put motor back into previous run mode
  }

  public int readCurrentMotorTargetPositionCounts() {
    return motor.getTargetPosition();
  }

  /* Power Functions */

  /**
   * Returns the stored power value.
   *
   * @return Stored power value
   */
  public double getStoredMotorPower() {
    return power;
  }

  /**
   * Set the motor power, but will do so only if the requested power exceeds the threshold
   * difference from the current power, unless the request is for 0.
   * This function will also clip motor power if it exceeds the range -1.0 to +1.0
   *
   * @param power Requested motor power
   */
  public void setPower(double power) {
    // Range check
    if (power > 1.0) {
      power = 1.0;
    }
    else if (power < -1.0) {
      power = -1.0;
    }

    setPowerNoClamping(power);
  }

  public void setPowerNoClamping(double power) {
    if (((power == 0.0) && (this.power != 0.0))
          || (Math.abs(power - this.power) > Constants.MOTOR_POWER_THRESHOLD_FOR_NEW_COMMAND)) {

      this.power = power;
      motor.setPower(this.power);
    }
  }

  /* CURRENT FUNCTIONS */
  public double getMotorCurrent(CurrentUnit currentUnit) {
    return motor.getCurrent(currentUnit);
  }

  public double getMotorCurrentAlert(CurrentUnit currentUnit) {
    return motor.getCurrentAlert(currentUnit);
  }

  public void setMotorCurrentAlert(double current, CurrentUnit currentUnit) {
    motor.setCurrentAlert(current, currentUnit);
  }

  public boolean isMotorOverCurrent() {
    return motor.isOverCurrent();
  }

  public double getMotorPower() {
    return motor.getPower();
  }

  /* POSITION FUNCTIONS */

  /**
   * Query the encoder for the current position (in counts)
   *
   * @return Current position (in counts)
   */
  public int getCurrentPosition() {
    if (encoderType == ENCODER.INTERNAL) {
      return motor.getCurrentPosition();
    }
    else {
      return (encoderDirection == DcMotorSimple.Direction.FORWARD)
          ? motor.getCurrentPosition()
          : -motor.getCurrentPosition();
    }
  }

  public int getTargetPosition() {
    return targetPosition;
  }

  /**
   * Set target position (counts)
   *
   * @param position Desired target position (counts)
   */
  public void setTargetPosition(int position) {
    if (position != targetPosition) {
      targetPosition = position;
      motor.setTargetPosition(targetPosition);
    }
  }

  /**
   * Set target position tolerance (counts)
   *
   * @param tolerance Desired tolerance (counts)
   */
  public void setTargetPositionTolerance(int tolerance) {
    motor.setTargetPositionTolerance(tolerance);
  }

  /* Velocity Functions */

  /**
   * Obtain the current velocity in rpm
   *
   * @return Current velocity (rpm)
   */
  public double getCurrentVelocityRPM() {
    return motor.getVelocity()
        / countsPerRev
        / internalGearReduction
        / externalGearReduction
        * 60.0;
  }

  public double getCurrentVelocityCPS() {
    return motor.getVelocity();
  }

  public void setTargetVelocity(double targetVelocityRPM, double targetVelocityToleranceRPM) {
    // Command expects counts per second
    motor.setVelocity(
        targetVelocityRPM / 60.0 * countsPerRev * internalGearReduction * externalGearReduction);
    this.targetVelocityRPM = targetVelocityRPM;
    this.targetVelocityToleranceRPM = targetVelocityToleranceRPM;
  }

  public double getTargetVelocityRPM() {
    return targetVelocityRPM;
  }

  public boolean isVelocityAtTarget() {
    double currentVelocityRPM = getCurrentVelocityRPM();
    if (targetVelocityRPM == 0.0) {
      return (currentVelocityRPM == 0.0);
    }
    else {
      return (currentVelocityRPM >= targetVelocityRPM)
              && (Math.abs(targetVelocityRPM - currentVelocityRPM)) <= targetVelocityToleranceRPM;
    }
  }

  public DcMotor.Direction getRunDirection() {
    return runDirection;
  }

  public boolean isBusy() {
    return motor.isBusy();
  }

  /**
   * Returns the Counts Per Revolution after internal and external gearing are accounted for
   *
   * @return CPR after internal and external gearing accounted for
   */
  public double getEffectiveCountsPerRev() {
    if (encoderType == ENCODER.INTERNAL) {
      return (countsPerRev * internalGearReduction * externalGearReduction);
    }
    else { // If using external encoder, assume it is on the output after all gearing
      return countsPerRev;
    }
  }

  public String getMotorName() {
    return motorName;
  }
}
