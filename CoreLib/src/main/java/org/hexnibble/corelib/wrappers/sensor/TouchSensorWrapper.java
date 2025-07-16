package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.hexnibble.corelib.misc.Msg;

public class TouchSensorWrapper extends CoreSensorWrapper<TouchSensor> {
  public TouchSensorWrapper(HardwareMap hwMap, String sensorName) {
    super(hwMap, TouchSensor.class, sensorName);

    Msg.log(
        getClass().getSimpleName(), "TouchSensorWrapper", "Creating touch sensor " + sensorName);
  }

  /**
   * Query whether the sensor is pressed.
   *
   * @return True if the sensor is pressed.
   */
  public boolean isPressed() {
    return sensor.isPressed();
  }
}
