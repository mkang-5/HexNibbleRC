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

   /**
   * Call this function to reset the sensor for a new OpMode.
   */
  public void reset() {}

  /**
   * Obtain sensor data as a string to be used for telemetry.
   * @return Sensor data (string)
   */
  public String getCachedSensorDataAsString() {
    return sensorName;
  }
}
