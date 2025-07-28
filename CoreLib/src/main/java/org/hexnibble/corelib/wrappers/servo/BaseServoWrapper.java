package org.hexnibble.corelib.wrappers.servo;

import android.util.Log;

import org.hexnibble.corelib.misc.Constants;

import java.util.HashMap;

public abstract class BaseServoWrapper {
  public enum RUN_DIRECTION {
    FORWARD,
    REVERSE
  }

  protected final RUN_DIRECTION runDirection;

  protected final String servoName;
  protected double targetPosition;
  protected final double absoluteMinimumPosition;
  protected final double absoluteMaximumPosition;
  private final double maxAngularRangeDegrees;
  protected double positionIncrement;

  // The default PWM range of the Rev hubs are 600 - 2400 microseconds.
  protected double PWMLowerPulse_us;
  protected double PWMUpperPulse_us;

  public enum SERVO_MODEL {
    Axon,
    GoBildaTorque,
    GoBildaSpeed,
    GoBildaSuperSpeed,
    GoBilda5Turn,
    HitecHLS12,
    Other
  }

  protected final SERVO_MODEL servoModel;

  protected final HashMap<String, Double> presetPositions;

  /**
   * Constructor for servo wrapper Toggle positions can also be specified.
   *
   * @param servoName Servo name
   * @param servoModel Servo model
   * @param absoluteMinimumPosition Minimum servo position: 0 to 1 for regular servo, -1 to +1 for
   *     continuous servo
   * @param absoluteMaximumPosition Maximum servo position: 0 to 1 for regular servo, -1 to +1 for
   *     continuous servo
   */
  public BaseServoWrapper(
      String servoName,
      SERVO_MODEL servoModel,
      double absoluteMinimumPosition,
      double absoluteMaximumPosition) {
    this.servoName = servoName;
    this.servoModel = servoModel;
    this.runDirection = RUN_DIRECTION.FORWARD;

    this.targetPosition = 0.0;

    if (absoluteMaximumPosition > 1.0) absoluteMaximumPosition = 1.0;
    if (absoluteMinimumPosition < 0.0) absoluteMinimumPosition = 0.0;
    this.absoluteMinimumPosition = absoluteMinimumPosition;
    this.absoluteMaximumPosition = absoluteMaximumPosition;

    if (this.servoModel == SERVO_MODEL.Axon) {
      this.positionIncrement = 1.0 / 360.0; // Half a degree each time
      this.maxAngularRangeDegrees = 352.0;
    } else if (this.servoModel == SERVO_MODEL.GoBilda5Turn) {
      this.positionIncrement = 0.001;
      this.maxAngularRangeDegrees = 360.0;
    } else {
      this.positionIncrement = 0.01;
      this.maxAngularRangeDegrees = 270.0;
    }

    this.presetPositions = new HashMap<>();
    this.presetPositions.put("MIN", absoluteMinimumPosition);
    this.presetPositions.put("MAX", absoluteMaximumPosition);
  }

  /** Call this function to initialize the servo. This will extend the PWM range as appropriate. */
  public void initialize() {
    Log.i(Constants.TAG, "BaseServoWrapper.j: Initializing servo " + servoName);
    extendPWMRange();
  }

  public abstract void reset();

  //    abstract protected void setServoRunDirection(RUN_DIRECTION servoRunDirection);
  public abstract void setPWMRange(double usPulseLower, double usPulseUpper);

  public void extendPWMRange() {
    switch (servoModel) {
      case Axon -> {
        PWMLowerPulse_us = 515; // Values <514 cause repeated/continuous servo movements
        PWMUpperPulse_us = 2495;
      }
      case GoBildaTorque, GoBildaSpeed, GoBildaSuperSpeed, GoBilda5Turn -> {
        PWMLowerPulse_us = 500;
        PWMUpperPulse_us = 2500;
      }
      case HitecHLS12 -> {
        PWMLowerPulse_us = 900;
        PWMUpperPulse_us = 2100;
      }
      default -> {
        PWMLowerPulse_us = 600;
        PWMUpperPulse_us = 2400;
      }
    }
    setPWMRange(PWMLowerPulse_us, PWMUpperPulse_us);
  }

  public abstract void disablePWM();

  public abstract void enablePWM();

  /**
   * Get the target position (or speed if CR) of the servo.
   *
   * @return Target position (or speed if CR)
   */
  public double getTargetPosition() {
    return targetPosition;
  }

  /**
   * Get the name of the servo.
   *
   * @return Name of the servo
   */
  public String getServoName() {
    return servoName;
  }

  /**
   * Set a regular servo to the specified position. This is for regular servos only.
   *
   * @param position Specified position (0.0 - 1.0).
   */
  public abstract void setServoPosition(double position);

  /**
   * Set a CR servo to the specified speed (-1.0 to +1.0). The underlying servo implementation
   * automatically scales this so be sure to provide the correct range.
   *
   * @param speed Specified speed (-1.0 to +1.0). Stop is 0.0
   */
  public abstract void setServoSpeed(double speed);

  /** Set the servo to the minimum position (or speed if CR). */
  public void setServoToMinPosition() {
    setServoPosition(absoluteMinimumPosition);
    targetPosition = absoluteMinimumPosition;
  }

  /** Set the servo to the maximum position (or speed if CR). */
  public void setServoToMaxPosition() {
    setServoPosition(absoluteMaximumPosition);
    targetPosition = absoluteMaximumPosition;
  }

  /** Increment the servo position (or speed if CR) up. */
  public void moveServoPositionUp() {
    targetPosition = getTargetPosition() + positionIncrement;
    setServoPosition(targetPosition);
  }

  /** Increment the servo position (or speed if CR) up. */
  public void moveServoPositionUp(float positionIncrement) {
    targetPosition = getTargetPosition() + positionIncrement;
    setServoPosition(targetPosition);
  }

  /** Decrement the servo position (or speed if CR) down. */
  public void moveServoPositionDown() {
    targetPosition = getTargetPosition() - positionIncrement;
    setServoPosition(targetPosition);
  }

  /** Decrement the servo position (or speed if CR) down. */
  public void moveServoPositionDown(float positionIncrement) {
    targetPosition = getTargetPosition() - positionIncrement;
    setServoPosition(targetPosition);
  }

  /**
   * Ensure that the specified position value is within the minimum/maximum range
   *
   * @param position Position value to check
   * @return Position that is within minimum/maximum range
   */
  protected double rangeCheckPosition(double position) {
    return (position < absoluteMinimumPosition)
        ? absoluteMinimumPosition
        : Math.min(position, absoluteMaximumPosition);
  }
}
