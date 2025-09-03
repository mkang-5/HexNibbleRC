package org.hexnibble.corelib.wrappers.controller;

import static org.hexnibble.corelib.misc.Constants.CONTROLLER_LEFT_TRIGGER_DEAD_ZONE;

/**
 * A wrapper class for the analog joysticks on the controller.
 */
public class AnalogStickWrapper {
  private float currentStickValue = 0.0f;
//  private float previousStickValue = 0.0f;
//  private boolean didStickValueChange = false;

  /**
   * Apply a dead zone around the 0 point of the joystick. This was help minimize small amounts of
   * spontaneous jitter from causing movements.
   * @param stickValue Joystick value (-1.0f to +1.0f but is not checked)
   * @param deadZone Dead zone around 0 (+/- the specified value)
   * @return Stick values within the dead zone become 0. Others are unchanged.
   */
  static float applySingleAxisStickDeadZone(float stickValue, float deadZone) {
    return (Math.abs(stickValue) <= deadZone) ? 0.0f : stickValue;
  }

  public void processStickValue(float rawStickValue) {
    float filteredRawStickValue =
        applySingleAxisStickDeadZone(rawStickValue, CONTROLLER_LEFT_TRIGGER_DEAD_ZONE);

//    didStickValueChange = false;
//
//    previousStickValue = currentStickValue;
    currentStickValue = filteredRawStickValue;
//    didStickValueChange = true;

/*
    // Only process stick values that have changed above the threshold amount or that have
    // changed to 0.

    if (filteredRawStickValue != previousStickValue) {
      if ((filteredRawStickValue == 0.0f)
              || (Math.abs(filteredRawStickValue - previousStickValue) > CONTROLLER_STICK_MOVEMENT_THRESHOLD))
      {
        previousStickValue = currentStickValue;
        currentStickValue = filteredRawStickValue;
        didStickValueChange = true;
      }
    }
 */
  }

//  public boolean didStickValueChange() {
//    return didStickValueChange;
//  }

  public float getCurrentFilteredStickValue() {
    return currentStickValue;
  }

//  public float getPreviousFilteredStickValue() {
//    return previousStickValue;
//  }
}
