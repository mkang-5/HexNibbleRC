package org.hexnibble.corelib.wrappers.controller;

public class ButtonWrapper {
  private ControllerWrapper.BUTTON_NAME buttonName;

  public enum BUTTON_STATE {
    NEWLY_PRESSED,
    NEWLY_RELEASED,
    CONTINUED_PRESSED,
    CONTINUED_RELEASED,
    DOWN
  }

  private BUTTON_STATE buttonState = BUTTON_STATE.CONTINUED_RELEASED; // Default state

  private boolean isButtonPressed = false;
  private boolean toggleState = false;

  public ButtonWrapper() {}

  public ButtonWrapper(ControllerWrapper.BUTTON_NAME buttonName) {
    this.buttonName = buttonName;
  }

  public ControllerWrapper.BUTTON_NAME getButtonName() {
    return buttonName;
  }

  /**
   * Process the state of the this button.
   *
   * @param isButtonPressed the button status obtained from the gamepad
   * @return Button state
   */
  public void processButtonState(boolean isButtonPressed) {
    if (isButtonPressed) {
      // If button was already pressed previously, then change state to continued pressed
      if (this.isButtonPressed) {
        buttonState = ButtonWrapper.BUTTON_STATE.CONTINUED_PRESSED;
      } else {
        this.isButtonPressed = true; // Store the newly pressed state
        toggleState = !toggleState; // Invert toggle state
        buttonState = ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED;
      }
    } else {
      if (this.isButtonPressed) { // Already pressed
        this.isButtonPressed = false; // Store the newly released state
        buttonState = ButtonWrapper.BUTTON_STATE.NEWLY_RELEASED;
      } else {
        buttonState = ButtonWrapper.BUTTON_STATE.CONTINUED_RELEASED;
      }
    }
  }

  /**
   * Queries whether the button is newly pressed. Be sure to call processButtonState first, if
   * needed.
   *
   * @return True if the button state is NEWLY_PRESSED
   */
  public boolean isButtonNewlyPressed() {
    return (buttonState == BUTTON_STATE.NEWLY_PRESSED);
  }

  /**
   * Queries whether the button is continued pressed. Be sure to call processButtonState first, if
   * needed.
   *
   * @return True if the button state is CONTINUED_PRESSED
   */
  public boolean isButtonContinuedPressed() {
    return (buttonState == BUTTON_STATE.CONTINUED_PRESSED);
  }

  public boolean isButtonDown() {
    return isButtonPressed;
  }

  /**
   * Retrieve the current button state.
   *
   * @return State of the button
   */
  public ButtonWrapper.BUTTON_STATE getButtonState() {
    return buttonState;
  }

  /**
   * Returns whether the button is toggled on or off.
   *
   * @return Toggle state as true/false
   */
  public boolean getToggleState() {
    return toggleState;
  }

  /**
   * Set whether the button is toggled on or off. This will also turn off button pressed.
   *
   * @param toggleState True/False whether the button is toggled on/off.
   */
  public void setToggleState(boolean toggleState) {
    this.toggleState = toggleState;
    isButtonPressed = false;
  }
}
