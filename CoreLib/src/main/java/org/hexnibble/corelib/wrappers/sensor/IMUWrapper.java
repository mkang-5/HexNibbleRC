package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.hexnibble.corelib.misc.Msg;

//public class IMUWrapper extends CoreSensorWrapper<IMU> implements IMUIface {
public class IMUWrapper extends CoreSensorWrapper<IMU> {
  private double currentIMUHeadingDegrees = 0.0;

  public IMUWrapper(
      HardwareMap hwMap,
      String sensorName,
      RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
      RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection) {
    super(hwMap, IMU.class, sensorName);

    Msg.log(
        getClass().getSimpleName(), "IMUWrapper", "Creating IMU " + sensorName
            + " with Hub Logo Direction = " + logoFacingDirection
            + ", Hub USB Direction = " + usbFacingDirection);

    if (!sensor.initialize(
        new IMU.Parameters(
            new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)))) {
      Msg.log(getClass().getSimpleName(), "IMUWrapper", "** IMU initialization failed.");
    }
  }

  /** Reset the IMU heading (yaw). */
  public void resetIMUHeading() {
    Msg.log(getClass().getSimpleName(), "resetIMUHeading", "Resetting IMU yaw/heading.");
    sensor.resetYaw();

    currentIMUHeadingDegrees = 0.0;
  }

  /**
   * Refresh the IMU heading from the actual sensor. Based on the right hand rule, the IMU heading
   * is an angle between +180 (CCW) and -180 (CW) degrees, with a zero position relative to when the
   * IMU was last initialized. Each read over I2C will take some time so make sure to do this only
   * once per loop. Use getStoredIMUHeadingDegrees() to obtain the most recently refreshed heading
   * without making a new call to the IMU.
   *
   * @return The newly read IMU Heading
   */
  public double readIMUHeading() {
    YawPitchRollAngles angles = sensor.getRobotYawPitchRollAngles();
    currentIMUHeadingDegrees = angles.getYaw(AngleUnit.DEGREES);

    return currentIMUHeadingDegrees;
  }

  public double getStoredIMUHeadingDegrees() {
    return currentIMUHeadingDegrees;
  }
}
