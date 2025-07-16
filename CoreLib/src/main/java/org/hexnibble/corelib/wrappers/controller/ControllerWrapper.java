package org.hexnibble.corelib.wrappers.controller;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.ArrayList;
import java.util.EnumMap;
import org.hexnibble.corelib.misc.Msg;

public class ControllerWrapper {
  public enum BUTTON_NAME {
    options, share,
    cross, square, triangle, circle,
    dpad_up, dpad_down, dpad_left, dpad_right,
    left_bumper, right_bumper,
    left_trigger, right_trigger,
    PS, touchpad,
    left_stick_button, right_stick_button,
    touchpad_finger_1, touchpad_finger_2
  }

  public enum ANALOG_STICK {
    left_stickX, left_stickY,
    right_stickX, right_stickY,
    left_trigger, right_trigger
  }

  private int shiftButtonState;
  public static final int SHARE_SHIFT = 0b0010;
  public static final int OPTION_SHIFT = 0b0001;
  public static final int NO_SHIFT = 0b0000;

  public enum SHIFT_BUTTON {
    SHARE,
    OPTIONS,
    NONE
  }

  private final Gamepad gamepadReference; // Store a reference to the underlying gamepad
  private final Gamepad gamepadCopy =
      new Gamepad(); // Store a copy of the gamepad state at a particular point in time

  private final EnumMap<ANALOG_STICK, AnalogStickWrapper> analogStickEnumMap =
      new EnumMap<>(ANALOG_STICK.class);
  private final ArrayList<AnalogStickToFunction[]> activeStickGroups = new ArrayList<>();

  private final EnumMap<BUTTON_NAME, ButtonWrapper> buttonEnumMap =
      new EnumMap<>(BUTTON_NAME.class);
  private final ArrayList<ButtonToFunction[]> activeButtonGroups = new ArrayList<>();

  static final float CONTROLLER_STICK_DEAD_ZONE_X = 0.05f;
  static final float CONTROLLER_STICK_DEAD_ZONE_Y = 0.05f;
  static final float CONTROLLER_LEFT_TRIGGER_DEAD_ZONE = 0.05f;
  static final float CONTROLLER_RIGHT_TRIGGER_DEAD_ZONE = 0.05f;

  // Shift Buttons
  private boolean optionShiftPressed;
  private boolean shareShiftPressed;

  public enum CONTROLLER_LED_COLOR {
    RED, GREEN, BLUE, YELLOW, CYAN, PURPLE, OFF
  }

  // Rumble
  Gamepad.RumbleEffect rumbleEffect;


  public ControllerWrapper(Gamepad gamepadReference) {
    this.gamepadReference = gamepadReference;

    for (BUTTON_NAME buttonName : BUTTON_NAME.values())
      buttonEnumMap.put(buttonName, new ButtonWrapper());

    for (ANALOG_STICK analogStick : ANALOG_STICK.values())
      analogStickEnumMap.put(analogStick, new AnalogStickWrapper());

    rumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 200)
            .addStep(0.0, 0.0, 100)
            .addStep(1.0, 1.0, 200)
            .build();
  }

  /**
   * Run this method at the beginning of each loop during the OpMode to save a copy of the
   * controller data. This prevents the data from changing during a single iteration of the loop.
   * After copying the controller data, the status of each analog stick and button will be updated.
   * Finally, any added button commands will be processed.
   */
  public void updateGamepadData() {
    gamepadCopy.copy(gamepadReference);

    // Update each analog stick
    analogStickEnumMap.forEach(
        (analogStickName, analogStickWrapper) ->
            analogStickWrapper.processStickValue(getRawStickValue(analogStickName)));

    // Run the associated function for any changed stick values
    activeStickGroups.forEach(this::processActiveStickGroup);

    // Update the button state of each button
    buttonEnumMap.forEach(
        (buttonName, buttonWrapper) ->
            buttonWrapper.processButtonState(isButtonPressed(buttonName)));

    optionShiftPressed = isButtonPressed(BUTTON_NAME.options);
    shareShiftPressed = isButtonPressed(BUTTON_NAME.share);

    shiftButtonState = (shareShiftPressed ? 1 : 0);
    shiftButtonState = (shiftButtonState << 1) + (optionShiftPressed ? 1 : 0);

    // Run the associated function for any active buttons
    activeButtonGroups.forEach(this::processActiveButtonGroup);
  }

  public void addActiveStickGroup(AnalogStickToFunction... stickGroup) {
    activeStickGroups.add(stickGroup);
  }

  private void processActiveStickGroup(AnalogStickToFunction[] stickGroup) {
    for (AnalogStickToFunction stickToFunction : stickGroup) {
      AnalogStickWrapper stickWrapper = analogStickEnumMap.get(stickToFunction.analogStickName());
      if (stickWrapper != null) {
        if (stickWrapper.didStickValueChange()) {
          stickToFunction.functionToRun().accept(stickWrapper.getCurrentFilteredStickValue());
          return; // Since an active stick was found in this group, no need to process any further
        }
      } else
        Msg.log(
            getClass().getSimpleName(),
            "processActiveStickGroup",
            "Null stickWrapper when looking for " + stickToFunction.analogStickName());
    }
  }

  public void addActiveButtonGroup(ButtonToFunction... buttonGroup) {
    activeButtonGroups.add(buttonGroup);
  }

  private void processActiveButtonGroup(ButtonToFunction[] buttonGroup) {
    for (ButtonToFunction buttonToFunction : buttonGroup) {
      ButtonWrapper buttonWrapper = buttonEnumMap.get(buttonToFunction.buttonName);
      if (buttonWrapper != null) {
        if (shiftButtonState == buttonToFunction.shiftButtonState) {
          if (((buttonToFunction.desiredButtonState == ButtonWrapper.BUTTON_STATE.DOWN)
                  && buttonWrapper.isButtonDown())
              || (buttonToFunction.desiredButtonState == buttonWrapper.getButtonState())) {
            buttonToFunction.functionToRun.run();
            // Since an active button was found in this group, no need to process any further
            return;
          }
        }
      } else
        Msg.log(
            getClass().getSimpleName(),
            "processActiveButtonGroup",
            "Null buttonWrapper when looking for " + buttonToFunction.buttonName);
    }
  }

  /**
   * Obtain the value of the left trigger. The trigger can also be used as a toggle button by adding
   * it as a button and using processButton.
   *
   * @return Value of the left trigger (0.0f - 1.0f).
   */
  public float getLeftTrigger() {
    return applySingleAxisStickDeadZone(
        gamepadCopy.left_trigger, CONTROLLER_LEFT_TRIGGER_DEAD_ZONE);
  }

  /**
   * Obtain the value of the right trigger. The trigger can also be used as a toggle button by
   * adding it as a button and using processButton.
   *
   * @return Value of the right trigger (0.0f - 1.0f).
   */
  public float getRightTrigger() {
    return applySingleAxisStickDeadZone(
        gamepadCopy.right_trigger, CONTROLLER_RIGHT_TRIGGER_DEAD_ZONE);
  }

  public float getLeftStickX() {
    return applySingleAxisStickDeadZone(gamepadCopy.left_stick_x, CONTROLLER_STICK_DEAD_ZONE_X);
  }

  public float getLeftStickY() {
    return applySingleAxisStickDeadZone(gamepadCopy.left_stick_y, CONTROLLER_STICK_DEAD_ZONE_Y);
  }

  public float getRightStickX() {
    return applySingleAxisStickDeadZone(gamepadCopy.right_stick_x, CONTROLLER_STICK_DEAD_ZONE_X);
  }

  public float getRightStickY() {
    return applySingleAxisStickDeadZone(gamepadCopy.right_stick_y, CONTROLLER_STICK_DEAD_ZONE_Y);
  }

  public float getRawStickValue(ANALOG_STICK analogStick) {
    return switch (analogStick) {
      case left_stickX -> gamepadCopy.left_stick_x;
      case left_stickY -> gamepadCopy.left_stick_y;
      case right_stickX -> gamepadCopy.right_stick_x;
      case right_stickY -> gamepadCopy.right_stick_y;
      case left_trigger -> gamepadCopy.left_trigger;
      case right_trigger -> gamepadCopy.right_trigger;
    };
  }

  public boolean isButtonPressed(BUTTON_NAME buttonName) {
    return switch (buttonName) {
      case options -> gamepadCopy.options;
      case share -> gamepadCopy.share;
      case cross -> gamepadCopy.cross;
      case square -> gamepadCopy.square;
      case triangle -> gamepadCopy.triangle;
      case circle -> gamepadCopy.circle;
      case dpad_up -> gamepadCopy.dpad_up;
      case dpad_down -> gamepadCopy.dpad_down;
      case dpad_left -> gamepadCopy.dpad_left;
      case dpad_right -> gamepadCopy.dpad_right;
      case left_bumper -> gamepadCopy.left_bumper;
      case right_bumper -> gamepadCopy.right_bumper;
      case left_trigger -> (gamepadCopy.left_trigger > CONTROLLER_LEFT_TRIGGER_DEAD_ZONE);
      case right_trigger -> (gamepadCopy.right_trigger > CONTROLLER_RIGHT_TRIGGER_DEAD_ZONE);
      case PS -> gamepadCopy.ps;
      case touchpad -> gamepadCopy.touchpad;
      case left_stick_button -> gamepadCopy.left_stick_button;
      case right_stick_button -> gamepadCopy.right_stick_button;
      case touchpad_finger_1 -> gamepadCopy.touchpad_finger_1;
      case touchpad_finger_2 -> gamepadCopy.touchpad_finger_2;
    };
  }

  public ButtonWrapper.BUTTON_STATE getButtonStatus(BUTTON_NAME buttonName) {
    ButtonWrapper buttonWrapper = buttonEnumMap.get(buttonName);

    if (buttonWrapper != null) return buttonWrapper.getButtonState();
    else return ButtonWrapper.BUTTON_STATE.CONTINUED_RELEASED;
  }

  /**
   * Query whether a button is currently newly pressed. This version defaults to No shift button. If
   * ButtonWrapper does not exist, the function will throw a Runtime Exception
   *
   * @param buttonName Name of the button
   * @return True/False
   */
  public boolean isButtonNewlyPressed(BUTTON_NAME buttonName) {
    return isButtonNewlyPressed(buttonName, SHIFT_BUTTON.NONE);
  }

  public boolean isButtonNewlyPressed(BUTTON_NAME buttonName, @NonNull SHIFT_BUTTON shiftButton) {
    ButtonWrapper buttonWrapper = buttonEnumMap.get(buttonName);
    if (buttonWrapper != null) {
      switch (shiftButton) {
        case SHARE -> {
          return shareShiftPressed && buttonWrapper.isButtonNewlyPressed();
        }
        case OPTIONS -> {
          return optionShiftPressed && buttonWrapper.isButtonNewlyPressed();
        }
        case NONE -> {
          return !shareShiftPressed && !optionShiftPressed && buttonWrapper.isButtonNewlyPressed();
        }
        default ->
            throw new RuntimeException(
                "Invalid shift button. Idk how this even happened unless you modified ControllerWrapper SHIFT_BUTTON enum.");
      }
    } else return false;
  }

  /**
   * Query whether a button is currently continued pressed. If ButtonWrapper does not exist, the
   * function will throw a Runtime Exception
   *
   * @param buttonName Name of the button
   * @return True/False
   */
  public boolean isButtonContinuedPressed(BUTTON_NAME buttonName) {
    return isButtonContinuedPressed(buttonName, SHIFT_BUTTON.NONE);
  }

  public boolean isButtonContinuedPressed(BUTTON_NAME buttonName, SHIFT_BUTTON shiftButton) {
    ButtonWrapper buttonWrapper = buttonEnumMap.get(buttonName);
    if (buttonWrapper != null) {
      switch (shiftButton) {
        case SHARE -> {
          return shareShiftPressed && buttonWrapper.isButtonContinuedPressed();
        }
        case OPTIONS -> {
          return optionShiftPressed && buttonWrapper.isButtonContinuedPressed();
        }
        case NONE -> {
          return !shareShiftPressed
              && !optionShiftPressed
              && buttonWrapper.isButtonContinuedPressed();
        }
        default ->
            throw new RuntimeException(
                "Invalid shift button. Idk how this even happened unless you modified ControllerWrapper SHIFT_BUTTON enum.");
      }
    } else return false;
  }

  public boolean isOptionShiftPressed() {
    return optionShiftPressed;
  }

  public boolean isShareShiftPressed() {
    return shareShiftPressed;
  }

  private float applySingleAxisStickDeadZone(float stickValue, float deadZone) {
    return (Math.abs(stickValue) <= deadZone) ? 0.0f : stickValue;
  }

  public void rumble(double rumble1Power, double rumble2Power, int duration_ms) {
    gamepadReference.rumble(rumble1Power, rumble2Power, duration_ms);
  }

  public void runRumbleEffect(Gamepad.RumbleEffect rumbleEffect) {
    gamepadReference.runRumbleEffect(rumbleEffect);
  }

  public void setLEDColor(CONTROLLER_LED_COLOR LEDColor) {
    switch (LEDColor) {
      case RED -> gamepadReference.setLedColor(1.0, 0.0, 0.0, -1);
      case YELLOW -> gamepadReference.setLedColor(1.0, 1.0, 0.0, -1);
      case GREEN -> gamepadReference.setLedColor(0.0, 1.0, 0.0, -1);
      case CYAN -> gamepadReference.setLedColor(0.0, 1.0, 1.0, -1);
      case BLUE -> gamepadReference.setLedColor(0.0, 0.0, 1.0, -1);
      case PURPLE -> gamepadReference.setLedColor(1.0, 0.0, 1.0, -1);
      case OFF -> gamepadReference.setLedColor(0.0, 0.0, 0.0, -1);
    }
  }

  public void runSavedRumbleEffect() {
    gamepadReference.runRumbleEffect(rumbleEffect);
  }
}
