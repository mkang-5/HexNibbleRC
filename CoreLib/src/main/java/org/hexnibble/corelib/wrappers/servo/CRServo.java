package org.hexnibble.corelib.wrappers.servo;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.hexnibble.corelib.misc.Msg;

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
      double minSpeed, double maxSpeed) {

    super(servoName, servoModel, minSpeed, maxSpeed);
    assert (minSpeed >= -1.0);
    assert (maxSpeed <= 1.0);

    if (hwMap == null) {
      throw new NullPointerException();
    }

    this.servo = hwMap.get(CRServoImplEx.class, servoName);
  }

  /**
   * Easy constructor for CRServo with default min/max as -1/+1 and servo off to start. Toggle
   * positions can also be specified.
   *
   * @param hwMap Hardware map
   * @param servoName Servo name
   * @param servoModel Servo model
   */
  public CRServo(
      HardwareMap hwMap,
      String servoName,
      SERVO_MODEL servoModel,
      RUN_DIRECTION servoRunDirection) {
    this(hwMap, servoName, servoModel, -1.0, 1.0);
  }

  @Override
  public void reset() {
    extendPWMRange();
    //        setServoRunDirection(runDirection);
  }

  /**
   * Set the run direction for the servo, FORWARD or REVERSE
   *
   * @param servoRunDirection Run direction of servo (FORWARD or REVERSE)
   */
  /*    @Override
      protected void setServoRunDirection(RUN_DIRECTION servoRunDirection) {
          if (servoRunDirection == RUN_DIRECTION.FORWARD) {
              servo.setDirection(DcMotorSimple.Direction.FORWARD);
          }
          else servo.setDirection(DcMotorSimple.Direction.REVERSE);
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

  @Override
  public void disablePWM() {
    servo.setPwmDisable();
  }

  @Override
  public void enablePWM() {
    servo.setPwmEnable();
  }

  @Override
  public void setServoPosition(double position) {
    // This function should not be used for a CRServo
    Msg.log(getClass().getSimpleName(), "setServoPosition", "setServoPosition should not be called on a CRServo.");
  }

  /**
   * Set a CR servo to the specified speed (-1.0 to +1.0). The underlying servo implementation
   * automatically scales this so be sure to provide the correct range.
   *
   * @param speed Specified speed (-1.0 to +1.0). Stop is 0.0
   */
  @Override
  public void setServoSpeed(double speed) {
    double checkedPosition = rangeCheckPosition(speed);

    servo.setPower(checkedPosition);
    targetPosition = checkedPosition;
  }
}
