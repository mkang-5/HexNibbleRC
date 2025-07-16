package org.hexnibble.corelib.wrappers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Deprecated(since = "5/25/25", forRemoval = true)
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
