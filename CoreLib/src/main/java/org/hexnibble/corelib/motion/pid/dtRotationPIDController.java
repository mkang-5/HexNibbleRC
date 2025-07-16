package org.hexnibble.corelib.motion.pid;

import org.hexnibble.corelib.misc.Field;

public class dtRotationPIDController extends PIDController {
  public enum ROTATION_DIRECTION {
    CLOCKWISE,
    COUNTERCLOCKWISE,
    SHORTEST
  }
  private final ROTATION_DIRECTION rotationDirection;

  // Angle errors less than this threshold at the beginning of this movement
  // will ignore the requested rotation direction
  private final double thresholdErrorForForcingTurnDirectionDegrees = 45.0;

  public dtRotationPIDController(
      double Ks,
      double Kp,
      double Ki,
      double Kd,
      double targetTolerance,
      ROTATION_DIRECTION rotationDirection) {
    super(Ks, Kp, Ki, Kd, targetTolerance);

    this.rotationDirection = rotationDirection;
  }

  public dtRotationPIDController(
      PIDSettings settings,
      double targetTolerance,
      ROTATION_DIRECTION rotationDirection) {
    super(settings, targetTolerance);

    this.rotationDirection = rotationDirection;
  }

  @Override
  public double calculateNewControlValue(double currentErrorRadians) {
    // This section is specific for spin turns

    // The current error will never be greater than 180 degrees (-180 to +180)

    double currentErrorToForceTurnDirection = currentErrorRadians;

    if (targetStateUpdated) { // This is true the very first time through
      if (rotationDirection == ROTATION_DIRECTION.CLOCKWISE) {
        if (currentErrorRadians > Math.toRadians(thresholdErrorForForcingTurnDirectionDegrees)) {
          currentErrorToForceTurnDirection = currentErrorRadians - (Math.PI * 2.0);
        }
      }
      else if (rotationDirection == ROTATION_DIRECTION.COUNTERCLOCKWISE) {
        // If the current error is negative, it means the current heading is rotated CCW relative to
        // the target heading. The shortest distance would thus usually be a CW turn, but a CCW turn
        // was requested. To force the CCW, 360 degrees is added to make a positive error.
        // This way, the correction to bring it back toward zero will result in a CCW turn.
        if (currentErrorRadians < -Math.toRadians(thresholdErrorForForcingTurnDirectionDegrees)) {
          currentErrorToForceTurnDirection = currentErrorRadians + (Math.PI * 2.0);
        }
      }

      // If the error to force a turn direction is >180 degrees, that value must be used.
      // Once the error is <180 degrees (PI), the targetStateUpdated flag is set to false.
      if ((currentErrorToForceTurnDirection > Math.PI)
          || (currentErrorToForceTurnDirection < -Math.PI)) {
        currentErrorRadians = currentErrorToForceTurnDirection;
      }
      else {
        targetStateUpdated = false;
      }
    }

    if (!targetStateUpdated) {
      currentErrorRadians = Field.enforceIMUHeadingRangeRadians(currentErrorRadians);
    }

    return super.calculateNewControlValue(currentErrorRadians);
  }
}
