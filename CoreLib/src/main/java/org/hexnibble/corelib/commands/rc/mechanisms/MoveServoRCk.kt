package org.hexnibble.corelib.commands.rc.mechanisms

import org.hexnibble.corelib.commands.rc.RCk
import org.hexnibble.corelib.wrappers.servo.RegularServo

/**
 * RC to move a servo.
 * @param servo Servo to move.
 * @param targetServoLocation Target position or degree value to send servo to.
 * @param servoUnit Specify whether sending servo to a position or specific degree. Defaults to DEGREE.
 * @param delayAfterMovementMs Delay to use after moving servo. Defaults to 200ms.
 * @param commandID The ID of the command.
 */
class MoveServoRCk @JvmOverloads constructor(
  private var servo: RegularServo,
  private val targetServoLocation: Double,
  private val servoUnit: ServoUnit = ServoUnit.DEGREES,
  delayAfterMovementMs: Int = 200,
  commandID: String = "MoveServoRC"
) : RCk(commandID, delayAfterMovementMs) {
  enum class ServoUnit {
    POSITION,
    DEGREES
  }

  override fun onStartCommand() {
    if (servoUnit == ServoUnit.POSITION) {
      servo.setServoPosition(targetServoLocation)
    } else {
      servo.servoPositionDegrees = targetServoLocation
    }
  }

  override fun processCommand() {
    // If using encoder and servo within 0.01 of target position or 2 of target degree, RC is done
    if (servo.isServoEncoderActive) {
      if (servoUnit == ServoUnit.POSITION) {
        if (servo.readServoPositionFromEncoder() in targetServoLocation - 0.01..targetServoLocation + 0.01) {
          commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
        }
      } else {
        if (servo.readServoPositionFromEncoderDegrees() in targetServoLocation - 2..targetServoLocation + 2) {
          commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
        }
      }
    }
  }
}