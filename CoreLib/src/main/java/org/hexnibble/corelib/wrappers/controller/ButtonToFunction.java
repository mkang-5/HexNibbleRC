package org.hexnibble.corelib.wrappers.controller;

public class ButtonToFunction {
  public final ControllerWrapper.BUTTON_NAME buttonName;
  public final ButtonWrapper.BUTTON_STATE desiredButtonState;
  public final int shiftButtonState;
  public final Runnable functionToRun;

  /**
   * Full constructor
   *
   * @param buttonName
   * @param desiredButtonState
   * @param shiftButton
   * @param functionToRun
   */
  public ButtonToFunction(
      ControllerWrapper.BUTTON_NAME buttonName,
      ButtonWrapper.BUTTON_STATE desiredButtonState,
      int shiftButton,
      Runnable functionToRun) {
    this.buttonName = buttonName;
    this.desiredButtonState = desiredButtonState;
    this.shiftButtonState = shiftButton;
    this.functionToRun = functionToRun;
  }

  /**
   * Simple constructor. This one assumes the button is newly pressed and no shift button is
   * pressed.
   *
   * @param buttonName
   * @param functionToRun
   */
  public ButtonToFunction(ControllerWrapper.BUTTON_NAME buttonName, Runnable functionToRun) {
    this(
        buttonName,
        ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED,
        ControllerWrapper.NO_SHIFT,
        functionToRun);
  }

  /**
   * Simple constructor for shift button. This one assumes the button is newly pressed.
   *
   * @param buttonName
   * @param functionToRun
   * @param shiftButton
   */
  public ButtonToFunction(
      ControllerWrapper.BUTTON_NAME buttonName, int shiftButton, Runnable functionToRun) {
    this(buttonName, ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED, shiftButton, functionToRun);
  }
}
