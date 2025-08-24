package org.hexnibble.corelib.wrappers.servo;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class RegularServo extends BaseServoWrapper {
  private final ServoImplEx servo;

  /**
   * Constructor providing angles.
   *
   * @param hwMap Hardware map
   * @param servoName Servo name
   * @param servoModel Servo model
   * @param encoderName Servo analog input (encoder) name
   * @param encoderDirection Servo encoder direction
   * @param absoluteMinimumPosition Absolute minimum servo position allowed
   * @param absoluteMaximumPosition Absolute maximum servo position allowed
   * @param lowerReferencePosition Minimum servo position: 0 to 1 for regular servo
   * @param angleAtLowerReferencePosition Angle (degrees) at minimum servo position
   * @param upperReferencePosition Maximum servo position: 0 to 1 for regular servo
   * @param angleAtUpperReferencePosition Angle (degrees) at maximum servo position
   */
  public RegularServo(HardwareMap hwMap, String servoName, SERVO_MODEL servoModel,
      String encoderName, DcMotorSimple.Direction encoderDirection,
      double absoluteMinimumPosition, double absoluteMaximumPosition,
      double lowerReferencePosition, double angleAtLowerReferencePosition,
      double upperReferencePosition, double angleAtUpperReferencePosition) {

    super(hwMap, servoName, servoModel, absoluteMinimumPosition, absoluteMaximumPosition,
          lowerReferencePosition, angleAtLowerReferencePosition,
          upperReferencePosition, angleAtUpperReferencePosition,
          encoderName, encoderDirection);

//    this.lowerReferencePosition = lowerReferencePosition;
//    this.angleAtLowerReferencePosition = angleAtLowerReferencePosition;
//    this.upperReferencePosition = upperReferencePosition;
//    this.angleAtUpperReferencePosition = angleAtUpperReferencePosition;
//    this.servoPositionPerDegree =
//        (upperReferencePosition - lowerReferencePosition)
//            / (angleAtUpperReferencePosition - angleAtLowerReferencePosition);

    this.servo = hwMap.get(ServoImplEx.class, servoName);

//    if (encoderName != null) {
//      this.servoEncoder = new AnalogInputWrapper(encoderName, hwMap);
//      this.encoderDirection = encoderDirection;
//    } else {
//      this.servoEncoder = null;
//      this.encoderDirection = null;
//    }
  }

  /**
   * Constructor providing parameters as positions (0 - 1; 0.5 is the middle). Toggle positions can
   * also be specified.
   *
   * @param hwMap Hardware map
   * @param servoName Servo name
   * @param servoModel Servo model
   * @param encoderName Servo analog input (encoder) name
   * @param encoderDirection Servo encoder direction
   * @param absoluteMinimumPosition Minimum servo position: 0 to 1 for regular servo
   * @param absoluteMaximumPosition Maximum servo position: 0 to 1 for regular servo
   */
  public RegularServo(HardwareMap hwMap, String servoName, SERVO_MODEL servoModel,
      String encoderName, DcMotorSimple.Direction encoderDirection,
      double absoluteMinimumPosition, double absoluteMaximumPosition) {

    super(hwMap, servoName, servoModel, absoluteMinimumPosition, absoluteMaximumPosition,
          encoderName, encoderDirection);

    servo = hwMap.get(ServoImplEx.class, servoName);

//    if (encoderName != null) {
//      servoEncoder = new AnalogInputWrapper(encoderName, hwMap);
//      this.encoderDirection = encoderDirection;
//    }
//    else {
//      servoEncoder = null;
//      this.encoderDirection = null;
//    }

//    lowerReferencePosition = 0.0;
//    angleAtLowerReferencePosition = 0.0;
//    upperReferencePosition = 0.0;
//    angleAtUpperReferencePosition = 0.0;
//
//    servoPositionPerDegree = 0.0;
  }

  @Override
  public void reset() {
    extendPWMRange();

    if (servoEncoder != null) {
      // The Axon servo analog output does not seem to start sending a voltage until the servo has
      // received a set command.
      // So when the servo is reinitialized, send a set position command to the most recent target
      // position.
      //            Log.i(TAG, " RegularServo.j: reinitialize - Servo encoder exists so setting the
      // servo position for " + servoName + " to " + startingPosition);
      //            setServoPosition(targetPosition);
    }
  }

  /*
      @Override
      protected void setServoRunDirection(RUN_DIRECTION servoRunDirection) {
          if (servoRunDirection == RUN_DIRECTION.FORWARD) {
              servo.setDirection(Servo.Direction.FORWARD);
          }
          else servo.setDirection(Servo.Direction.REVERSE);
      }
  */
  /**
   * Set the PWM range for this servo.
   *
   * @param usPulseLower Lower bound of pulse width (microseconds)
   * @param usPulseUpper Upper bound of pulse width (microseconds)
   */
  @Override
  public void setPWMRange(double usPulseLower, double usPulseUpper) {
    servo.setPwmRange(new PwmControl.PwmRange(PWMLowerPulse_us, PWMUpperPulse_us));
  }

  public PwmControl.PwmRange getPWMRange() {
    return servo.getPwmRange();
  }

  @Override
  public void disablePWM() {
    servo.setPwmDisable();
  }

  @Override
  public void enablePWM() {
    servo.setPwmEnable();
  }

  /**
   * Read the servo's position from the attached encoder, if present.
   *
   * @return Servo position (0 - 1). Returns 0 if there is no encoder.
   */
  public double readServoPositionFromEncoder() {
    // Divide voltage by 3.3 V (this is the max value) to normalize to a value from 0 - 1.
    if (servoEncoder != null) {
      double position = servoEncoder.getVoltage() / 3.3;
      return (encoderDirection == DcMotorSimple.Direction.FORWARD) ? position : (1.0 - position);
    } else return targetPosition;
  }

  /**
   * Read the servo's position from the attached encoder, if present, and convert to degrees.
   *
   * @return Servo position, in degrees. Returns 0 if there is no encoder.
   */
  public double readServoPositionFromEncoderDegrees() {
    //        // Multiply servo position by 360 degrees to get the servo position in degrees.
    //        return (180.0 - (readServoPosition() * 360.0));

    return convertServoPositionToDegrees(readServoPositionFromEncoder());
  }

  //    public double getServoTargetPositionDegrees() {
  //        return ((targetPosition - 0.5) * maxAngularRangeDegrees) + degreesOffsetAtMidpoint;
  //    }

  /**
   * Set a regular servo to the specified position. This is for regular servos only.
   *
   * @param point Specified position (0.0 - 1.0).
   */
  @Override
  public void setServoPoint(double point) {
    double checkedPosition = rangeCheckPosition(point);

    if (checkedPosition != targetPosition) {
      servo.setPosition(checkedPosition);
      targetPosition = checkedPosition;
    }
  }

  public void setServoPositionDegrees(double targetAngleDegrees) {
    setServoPoint(
        lowerReferencePosition
            + (targetAngleDegrees - angleAtLowerReferencePosition) * servoPositionPerDegree);
  }

  public double getServoPositionDegrees() {
    return convertServoPositionToDegrees(getTargetPosition());
  }

  private double convertServoPositionToDegrees(double position) {
    //        Log.i(TAG, "ServoTargetPosition=" + getTargetPosition() + ", position/deg=" +
    // servoPositionPerDegree + ", angleAtLowerPosition=" + angleAtLowerPosition);
    return ((position - lowerReferencePosition) / servoPositionPerDegree)
        + angleAtLowerReferencePosition;
  }

  /**
   * Query whether an encoder is active for this servo
   *
   * @return True/False whether the servo encoder exists.
   */
  public boolean isServoEncoderActive() {
    return (servoEncoder != null);
  }
}
