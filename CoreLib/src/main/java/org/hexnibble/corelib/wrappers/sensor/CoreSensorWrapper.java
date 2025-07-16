package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CoreSensorWrapper<T> {
  protected final T sensor;
  protected final HardwareMap hwMap;
  protected final String sensorName;

  public CoreSensorWrapper(HardwareMap hwMap, Class<? extends T> sensorType, String sensorName) {
    if (hwMap == null) {
      throw new NullPointerException();
    } else this.hwMap = hwMap;

    sensor = hwMap.get(sensorType, sensorName);
    this.sensorName = sensorName;
  }

  public String getSensorName() {
    return sensorName;
  }

  public T getSensor() {
    return sensor;
  }

  public void reset() {}
}
