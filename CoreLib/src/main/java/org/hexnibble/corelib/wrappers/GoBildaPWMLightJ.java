package org.hexnibble.corelib.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Deprecated(since = "5/25/25", forRemoval = true)
public class GoBildaPWMLightJ {
  private final ServoImplEx light;

  public GoBildaPWMLightJ(HardwareMap hwMap, String pwmLightName) {
    light = hwMap.get(ServoImplEx.class, pwmLightName);
  }

  private void setPWM() {
    // The default PWM range of the Rev hubs are 600 - 2400 microseconds.
    light.setPwmRange(new PwmControl.PwmRange(500, 2500));
  }

  public void initialize() {
    setPWM();
  }

  public void reinitialize() {
    setPWM();
  }

  public void turnLightOff() {
    light.setPosition(0.0);
  }

  public void turnLightOn() {
    light.setPosition(1.0);
  }

  public void setLightBrightness(double brightness) {
    light.setPosition(brightness);
  }
}
