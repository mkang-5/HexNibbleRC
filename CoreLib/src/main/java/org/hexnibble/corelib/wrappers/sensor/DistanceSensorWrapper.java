package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hexnibble.corelib.misc.Msg;

public class DistanceSensorWrapper extends CoreSensorWrapper<Rev2mDistanceSensor> {
  private boolean
      isDistanceSensorFunctional; // Used especially for instances when the distance sensor goes out

  // of commission due to ESD

  public DistanceSensorWrapper(HardwareMap hwMap, String sensorName) {
    super(hwMap, Rev2mDistanceSensor.class, sensorName);

    if (sensor == null) {
      isDistanceSensorFunctional = false;
      Msg.log(getClass().getSimpleName(), "Constructor", "Distance sensor was not found.");
    } else isDistanceSensorFunctional = true;
  }

  @Override
  public void reset() {
    if (sensor != null) isDistanceSensorFunctional = true;
  }

  /**
   * Obtain the distance sensor reading. A reading will only be returned if the sensor has not sent
   * back a fake distance (65535). If this ever occurs, the sensor will not be interrogated any more
   * to avoid prolonged loop times, and the fake distance will always be sent back.
   *
   * @return
   */
  public double getDistanceSensorReading() {
    if (isDistanceSensorFunctional) {
      final double distance_mm = sensor.getDistance(DistanceUnit.MM);
      if (distance_mm
          == 65535) { // Look for the fake distance value used when the sensor is not working
        // properly
        Msg.log(
            getClass().getSimpleName(),
            "getDistanceSensorReading",
            "Distance sensor appears to have failed because it is sending back readings of 65535");
        isDistanceSensorFunctional = false;
      } else return distance_mm;
    }
    return 65535;
  }
}
