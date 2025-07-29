package org.hexnibble.corelib.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AnalogInputWrapperJ {
  AnalogInput analogSensor;

  public AnalogInputWrapperJ(String sensorName, HardwareMap hwMap) {
    if (hwMap == null) {
      throw new NullPointerException();
    }

    analogSensor = hwMap.get(AnalogInput.class, sensorName);
  }

  public double getVoltage() {
    return analogSensor.getVoltage();
  }
}
