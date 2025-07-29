package org.hexnibble.corelib.commands.rc.mechanisms;

import org.hexnibble.corelib.commands.rc.RCk;
import org.hexnibble.corelib.wrappers.servo.RegularServo;

// TODO: ADD SUPPORT FOR USING ENCODERS ON SERVO TO DETERMINE WHETHER GOT TO END POSITION
@Deprecated(since = "5/25/25", forRemoval = true)
public class MoveServoRCJ extends RCk {
  private final RegularServo servo;
  private final double targetServoLocation;

  public enum ServoUnit {
    POSITION,
    DEGREES
  }

  private final ServoUnit servoUnit;

  /**
   * Constructor for sending servo to specified degree or position
   *
   * @param servo Servo to move
   * @param targetServoLocation Target position or degree value to send servo to
   * @param unit Specify whether sending servo to a position or specific degree
   * @param delayAfterMovement_ms Delay after moving the servo to the target position
   */
  public MoveServoRCJ(
      String servoName,
      RegularServo servo,
      double targetServoLocation,
      ServoUnit unit,
      int delayAfterMovement_ms) {
    super(servoName, delayAfterMovement_ms);
    this.servo = servo;
    this.servoUnit = unit;
    this.targetServoLocation = targetServoLocation;
  }

  /**
   * Constructor for sending servo to specified degree
   *
   * @param servo Servo to move
   * @param targetServoPositionDegrees Target degree value to send servo to
   * @param delayAfterMovement_ms Delay after moving the servo to the target position
   */
  public MoveServoRCJ(
      String servoName,
      RegularServo servo,
      double targetServoPositionDegrees,
      int delayAfterMovement_ms) {
    this(servoName, servo, targetServoPositionDegrees, ServoUnit.DEGREES, delayAfterMovement_ms);
  }

  @Override
  protected void onStartCommand() {
    if (servoUnit == ServoUnit.POSITION) servo.setServoPosition(targetServoLocation);
    else servo.setServoPositionDegrees(targetServoLocation);
  }

  @Override
  protected void processCommand() {}
}
