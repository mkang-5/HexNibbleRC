package org.hexnibble.corelib.wrappers.controller;

import org.hexnibble.corelib.misc.FloatConsumer;

public record AnalogStickToFunction(ControllerWrapper.ANALOG_STICK analogStickName, FloatConsumer functionToRun) {
  public AnalogStickToFunction {
  }
}
