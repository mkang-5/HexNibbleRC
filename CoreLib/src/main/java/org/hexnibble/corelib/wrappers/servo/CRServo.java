package org.hexnibble.corelib.wrappers.servo;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

public class CRServo extends BaseServoWrapper {
  private final CRServoImplEx servo;

  /**
   * Constructor providing parameters as speeds (-1 to 1; 0.0 is stopped). Toggle positions can also
   * be specified.
   *
   * @param hwMap Hardware map
   * @param servoName Servo name
   * @param servoModel Servo model
   * @param minSpeed Minimum servo position: -1 to 1 for continuous rotation servo
   * @param maxSpeed Maximum servo position: -1 to 1 for continuous rotation servo
   */
  public CRServo(
      HardwareMap hwMap, String servoName, SERVO_MODEL servoModel,
      double minSpeed, double maxSpeed,
      String encoderName, DcMotorSimple.Direction encoderDirection) {

    super(hwMap, servoName, servoModel, minSpeed, maxSpeed, encoderName, encoderDirection);
    assert (minSpeed >= -1.0);
    assert (maxSpeed <= 1.0);

    this.servo = hwMap.get(CRServoImplEx.class, servoName);
  }

  /**
   * Easy constructor for CRServo with default min/max as -1/+1, servo off to start, and no encoder.
   * Toggle positions can also be specified.
   *
   * @param hwMap Hardware map
   * @param servoName Servo name
   * @param servoModel Servo model
   */
  public CRServo(
      HardwareMap hwMap,
      String servoName,
      SERVO_MODEL servoModel) {
    this(hwMap, servoName, servoModel, -1.0, 1.0, null, null);
  }

  @Override
  public void reset() {
    extendPWMRange();
    //        setServoRunDirection(runDirection);
  }

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

  @Override
  public void disablePWM() {
    servo.setPwmDisable();
  }

  @Override
  public void enablePWM() {
    servo.setPwmEnable();
  }

  /**
   * Set a CR servo to the specified speed (-1.0 to +1.0). The underlying servo implementation
   * automatically scales this so be sure to provide the correct range.
   *
   * @param point Specified speed (-1.0 to +1.0). Stop is 0.0
   */
  @Override
  public void setServoPoint(double point) {
    double checkedPosition = rangeCheckPosition(point);

    if (checkedPosition != targetPosition) {
      servo.setPower(checkedPosition);
      targetPosition = checkedPosition;
    }
  }
}
