package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hexnibble.corelib.misc.Msg;

public class RevDistanceSensorWrapper extends CoreSensorWrapper<Rev2mDistanceSensor> {
  private double cachedDistanceReading;

  // Used especially for instances when the distance sensor goes out of commission due to ESD
  private boolean isDistanceSensorFunctional;

  public RevDistanceSensorWrapper(HardwareMap hwMap, String sensorName) {
    super(hwMap, Rev2mDistanceSensor.class, sensorName);

    if (sensor == null) {
      isDistanceSensorFunctional = false;
      cachedDistanceReading = 65535.0;
      Msg.log(getClass().getSimpleName(), "Constructor", "Distance sensor was not found.");
    }
    else {
      isDistanceSensorFunctional = true;
      cachedDistanceReading = 0.0;
    }
  }

  @Override
  public void reset() {
    if (sensor != null) {
      isDistanceSensorFunctional = true;
      cachedDistanceReading = 0.0;
    }
  }

  /**
   * Obtain a fresh distance sensor reading. A reading will only be returned if the sensor has not sent
   * back a fake distance (65535). If this ever occurs, the sensor will not be interrogated any more
   * to avoid prolonged loop times, and the fake distance will always be sent back.
   *
   * @return
   */
  public double getDistanceSensorReading() {
    if (isDistanceSensorFunctional) {
      cachedDistanceReading = sensor.getDistance(DistanceUnit.MM);

      // Look for the fake distance value used when the sensor is not workingproperly
      if (cachedDistanceReading == 65535) {
        Msg.log(getClass().getSimpleName(), "getDistanceSensorReading",
              "Distance sensor appears to have failed because it is sending back 65535");
        isDistanceSensorFunctional = false;
      }
    }
    return cachedDistanceReading;
  }

  @Override
  public String getCachedSensorDataAsString() {
    return sensorName + ": " + cachedDistanceReading + " mm";
  }
}
