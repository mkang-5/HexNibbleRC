package org.hexnibble.corelib.robot_system;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.sensor.TouchSensorWrapper;

/** This class represents a linear mechanism, such as a lead screw or slide. */
public class LinearMechanism {
  public enum LINEAR_SYSTEM_TYPE {
    LEAD_SCREW,
    SINGLE_MOTOR_SLIDE,
    DUAL_MOTOR_SLIDE
  }

  public enum RIGGING_STYLE {
    CONTINUOUS,
    CASCADE
  }

  private final BaseMotorWrapper motor1, motor2;

  private TouchSensorWrapper minTouchSensor;

  private int encoderCountOffsetToZeroValue;

  private double currentMotor1TargetPosition_mm, currentMotor2TargetPosition_mm = 0.0;
  private final double minPosition_mm, maxPosition_mm, countsPer_mm, minPositionToStopMotor_mm;

  /**
   * Constructor for LinearMechanism
   *
   * @param hwMap Hardware map variable
   * @param slideMotorName Name of the slide motor specified in the driver station configuration *
   * @param motorModel Model of the motor used for the slide
   * @param motorRunDirection Run direction of motor
   * @param externalGearReduction External gear reduction
   * @param minPosition_mm Minimum position (mm) of the linear mechanism
   * @param maxPosition_mm Maximum position (mm) of the linear mechanism
   * @param minPositionToStopMotor_mm Minimum position in mm to stop motor
   * @param systemType Linear system type
   * @param outputTravel_mm If lead screw, specify the linear travel/revolution of the screw.
   *     GoBilda lead screws are 8 mm of linear travel per revolution of the screw If slide, specify
   *     the diameter of the output (whether pulley, wheel, gear, etc)
   */
  public LinearMechanism(
      HardwareMap hwMap,
      String slideMotorName,
      BaseMotorWrapper.MOTOR_MODEL motorModel,
      DcMotor.Direction motorRunDirection,
      double externalGearReduction,
      double minPosition_mm,
      double maxPosition_mm,
      double minPositionToStopMotor_mm,
      LINEAR_SYSTEM_TYPE systemType,
      double outputTravel_mm) {
    motor1 =
        new BaseMotorWrapper(
            hwMap,
            slideMotorName,
            motorModel,
            motorRunDirection,
            DcMotor.RunMode.RUN_TO_POSITION,
            BaseMotorWrapper.ENCODER.INTERNAL,
            motorRunDirection,
            externalGearReduction);
    motor2 = null;

    this.minPosition_mm = minPosition_mm;
    this.maxPosition_mm = maxPosition_mm;

    this.minPositionToStopMotor_mm = minPositionToStopMotor_mm;

    if (systemType == LINEAR_SYSTEM_TYPE.LEAD_SCREW) {
      countsPer_mm = motor1.getEffectiveCountsPerRev() / outputTravel_mm;
    } else {
      countsPer_mm = motor1.getEffectiveCountsPerRev() / (outputTravel_mm * Math.PI);
    }

    encoderCountOffsetToZeroValue = 0;
  }

  /**
   * Constructor for LinearMechanism with two motors
   *
   * @param hwMap Hardware map variable
   * @param slideMotorName Name of the slide motor specified in the driver station configuration *
   * @param motorModel Model of the motor used for the slide. This should be the motor that provides
   *     positive encoder readings for positive movement of the system.
   * @param motorRunDirection Run direction of motor
   * @param slideMotorName2 Name of the second slide motor specified in the driver station
   *     configuration *
   * @param motor2RunDirection Run direction of motor 2
   * @param externalGearReduction External gear reduction
   * @param minPosition_mm Minimum position (mm) of the linear mechanism
   * @param maxPosition_mm Maximum position (mm) of the linear mechanism
   * @param minPositionToStopMotor_mm Minimum position in mm to stop motor
   * @param systemType Linear system type
   * @param outputTravel_mm If lead screw, specify the linear travel/revolution of the screw.
   *     GoBilda lead screws are 8 mm of linear travel per revolution of the screw If slide, specify
   *     the diameter of the output (whether pulley, wheel, gear, etc)
   */
  public LinearMechanism(
      HardwareMap hwMap,
      String slideMotorName,
      BaseMotorWrapper.MOTOR_MODEL motorModel,
      DcMotor.Direction motorRunDirection,
      String slideMotorName2,
      DcMotor.Direction motor2RunDirection,
      double externalGearReduction,
      double minPosition_mm,
      double maxPosition_mm,
      double minPositionToStopMotor_mm,
      LINEAR_SYSTEM_TYPE systemType,
      double outputTravel_mm) {
    motor1 =
        new BaseMotorWrapper(
            hwMap,
            slideMotorName,
            motorModel,
            motorRunDirection,
            DcMotor.RunMode.RUN_TO_POSITION,
            BaseMotorWrapper.ENCODER.INTERNAL,
            motorRunDirection,
            externalGearReduction);
    motor2 =
        new BaseMotorWrapper(
            hwMap,
            slideMotorName2,
            motorModel,
            motor2RunDirection,
            DcMotor.RunMode.RUN_TO_POSITION,
            BaseMotorWrapper.ENCODER.INTERNAL,
            motor2RunDirection,
            externalGearReduction);

    this.minPosition_mm = minPosition_mm;
    this.maxPosition_mm = maxPosition_mm;

    this.minPositionToStopMotor_mm = minPositionToStopMotor_mm;

    if (systemType == LINEAR_SYSTEM_TYPE.LEAD_SCREW) {
      countsPer_mm = motor1.getEffectiveCountsPerRev() / outputTravel_mm;
    } else {
      countsPer_mm = motor1.getEffectiveCountsPerRev() / (outputTravel_mm * Math.PI);
    }

    encoderCountOffsetToZeroValue = 0;
  }

  /**
   * Call this function to reset motors when restarting an OpMode. It will revert back to the most
   * recent stored values.
   */
  public void reset() {
    motor1.reset();

    if (motor2 != null) {
      motor2.reset();
    }
  }

  public void associateTouchSensor(TouchSensorWrapper touchSensor) {
    minTouchSensor = touchSensor;
  }

  public boolean isTouchSensorPressed() {
    if (minTouchSensor != null) {
      return minTouchSensor.isPressed();
    } else return false;
  }

  /**
   * Move to a specified distance (based on lead distance/distance per rotation) using specified
   * power.
   *
   * @param position_mm Target position (mm)
   * @param motorPower Motor power for this movement (0 - 1)
   * @param enforceLimits Enforce range limits on the movement. This can be set false if the encoder
   *     becomes out of sync (e.g. if a belt skips)
   * @return New target position (mm), which may be different from the requested target if enforcing
   *     limits and it was out of the range.
   */
  public double moveToPosition_mm(double position_mm, double motorPower, boolean enforceLimits) {
    // Check limits
    if (enforceLimits) {
      if (position_mm < minPosition_mm) {
        position_mm = minPosition_mm;
      } else if (position_mm > maxPosition_mm) {
        Msg.logIfDebug(
            getClass().getSimpleName(),
            "moveToPosition_mm",
            "Target position="
                + position_mm
                + ", but max targetCount="
                + maxPosition_mm
                + " so capping movement.");
        position_mm = maxPosition_mm;
      }
    }

    int targetCount =
        ((int) (position_mm * countsPer_mm))
            + encoderCountOffsetToZeroValue; // Convert targetCount distance to encoder counts

    motor1.setTargetPosition(targetCount);
    motor1.setPower(Math.abs(motorPower));

    if (motor2 != null) {
      motor2.setTargetPosition(targetCount);
      motor2.setPower(Math.abs(motorPower));
    }

    return position_mm;
  }

  /**
   * Move toward the minimum distance (retraction).
   *
   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
   * @noinspection ReassignedVariable
   */
  public void moveToMinPosition(double motorPower) {
    double targetPosition_mm = minPosition_mm;
    boolean enforceLimits = true;
    double currentPosition_mm = getCurrentPositionMotor1_mm();

    // If a touch sensor is being used, then check whether it is on
    if (minTouchSensor != null) {
      if (minTouchSensor.isPressed()) {
        Msg.logIfDebug(
            getClass().getSimpleName(),
            "moveToMinPosition",
            "Touch sensor pressed so setting zero position count offset.");
        setEncoderZeroPositionCountOffset();
      } else {
        // If the touch sensor is not pressed but the current position is less than the minimum
        // position, then this means the encoder is out of sync and a forced movement should occur
        // If the current position is greater than the minimum position, then the slide should be
        // moving in.
        if (currentPosition_mm <= minPosition_mm) {
          enforceLimits = false;
          targetPosition_mm = currentPosition_mm - 100.0;
        }
      }
    }

    currentMotor1TargetPosition_mm =
        moveToPosition_mm(targetPosition_mm, motorPower, enforceLimits);

    if (motor2 != null) {
      currentMotor2TargetPosition_mm = currentMotor1TargetPosition_mm;
    }
  }

  /**
   * Move to the max distance (extension).
   *
   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
   */
  public void moveToMaxPosition(double motorPower) {
    currentMotor1TargetPosition_mm = moveToPosition_mm(maxPosition_mm, motorPower, true);

    if (motor2 != null) {
      currentMotor2TargetPosition_mm = currentMotor1TargetPosition_mm;
    }
  }

  /**
   * Call when the touch sensor is pressed, to obtain the motor count value and use as an offset for
   * the zero (minimum) position.
   */
  public void setEncoderZeroPositionCountOffset() {
    int count = motor1.getCurrentPosition();
    encoderCountOffsetToZeroValue = count;
    Msg.log(
        getClass().getSimpleName(),
        "setEncoderZeroPositionCountOffset",
        "Setting zeroOffset=" + count);
  }

  /**
   * Stop the mechanism at the current position. If power is set to zero, it may move based on
   * gravity depending on its weight and gearing. If power is left as is, then the motor will remain
   * running to try and maintain the position. There is also a check of any associated encoder to
   * check whether it is in--if so, the encoder can be zeroed.
   *
   * @param setPowerToZero If true, will set the Power to 0.
   */
  public void stopMechanism(boolean setPowerToZero) {
    motor1.setTargetPosition(motor1.getCurrentPosition());
    currentMotor1TargetPosition_mm = getCurrentPositionMotor1_mm();

    if (motor2 != null) {
      motor2.setTargetPosition(motor2.getCurrentPosition());
      currentMotor2TargetPosition_mm = getCurrentPositionMotor2_mm();
    }

    if (currentMotor1TargetPosition_mm <= minPositionToStopMotor_mm) {
      if (setPowerToZero) {
        motor1.setPower(0.0);
        if (motor2 != null) {
          motor2.setPower(0.0);
        }
      }
    }

    // If a touch sensor is being used, then check whether it is on
    if (minTouchSensor != null) {
      if (minTouchSensor.isPressed()) {
        Msg.logIfDebug(
            getClass().getSimpleName(),
            "stopMechanism",
            "Touch sensor pressed so setting zero position count offset.");
        setEncoderZeroPositionCountOffset();
      }
    }
  }

  public void setSystemMotorPower(double motorPower) {
    motor1.setPower(motorPower);

    if (motor2 != null) {
      motor2.setPower(motorPower);
    }
  }

  public void setSystemMotorRunMode(DcMotor.RunMode runMode) {
    motor1.setRunMode(runMode);

    if (motor2 != null) {
      motor2.setRunMode(runMode);
    }
  }

  public void setSystemMotorBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
    motor1.setBrakeMode(brakeMode);

    if (motor2 != null) {
      motor2.setBrakeMode(brakeMode);
    }
  }

  /**
   * Retrieve the current position of motor (mm), calculated using counts retrieved from the motor
   *
   * @return Current position of motor (mm)
   */
  public double getCurrentPositionMotor1_mm() {
    return getCurrentMotorPosition_mm(1);
  }

  public double getCurrentPositionMotor2_mm() {
    return getCurrentMotorPosition_mm(2);
  }

  private double getCurrentMotorPosition_mm(int motor) {
    double currentPositionCounts =
        switch (motor) {
          case 1 -> motor1.getCurrentPosition();
          case 2 -> motor2.getCurrentPosition();
          default -> throw new IllegalStateException("Unexpected value: " + motor);
        };

    double revisedCurrentPositionCounts = currentPositionCounts - encoderCountOffsetToZeroValue;

    return revisedCurrentPositionCounts / countsPer_mm;
  }

  /**
   * Retrieve the current target position of the specified motor, in mm
   *
   * @return Current target position of the specified motor, in mm
   */
  public double getCurrentTargetPositionMotor1_mm() {
    return currentMotor1TargetPosition_mm;
  }

  public double getCurrentTargetPositionMotor2_mm() {
    return currentMotor2TargetPosition_mm;
  }

  /**
   * Retrieve the current draw of the specified motor
   *
   * @return motor current in mA
   */
  public double getMotor1Current() {
    return motor1.getMotorCurrent(CurrentUnit.MILLIAMPS);
  }

  public double getMotor2Current() {
    return motor2.getMotorCurrent(CurrentUnit.MILLIAMPS);
  }

  /** Reset the slide encoder. This does not reset the target position. */
  public void resetEncoder() {
    motor1.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor1.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    if (motor2 != null) {
      motor2.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor2.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    encoderCountOffsetToZeroValue = 0;
  }

  public void setTargetPositionTolerance_mm(int tolerance_mm) {
    motor1.setTargetPositionTolerance((int) (countsPer_mm * tolerance_mm));

    if (motor2 != null) {
      motor2.setTargetPositionTolerance((int) (countsPer_mm * tolerance_mm));
    }
  }

  public boolean isSystemMotorBusy() {
    return motor1.isBusy();
  }
}
