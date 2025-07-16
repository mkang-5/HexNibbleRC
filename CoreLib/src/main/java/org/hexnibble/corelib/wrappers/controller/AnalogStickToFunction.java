package org.hexnibble.corelib.wrappers.controller;

import org.hexnibble.corelib.misc.FloatConsumer;

public record AnalogStickToFunction(ControllerWrapper.ANALOG_STICK analogStickName, FloatConsumer functionToRun) {
  /**
   * Simple constructor. This one assumes the button is newly pressed and no shift button is
   * pressed.
   *
   * @param analogStickName
   * @param functionToRun
   */
  public AnalogStickToFunction {
  }
}
