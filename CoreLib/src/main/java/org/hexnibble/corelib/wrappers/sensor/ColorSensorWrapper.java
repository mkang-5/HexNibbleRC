package org.hexnibble.corelib.wrappers.sensor;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.hexnibble.corelib.misc.Msg;

// Wrapper class for Rev Color Sensor v3. These are I2C devices.
public class ColorSensorWrapper extends CoreSensorWrapper<RevColorSensorV3> {
  private double lastDistanceSensorReading_mm;

  /**
   * Color sensors (especially the REV Color Sensor V3) can give very low values (depending on the
   * lighting conditions), which only use a small part the 0-1 range available for the red, green,
   * and blue values. You can give the sensor a gain value, which will be multiplied by the sensor's
   * raw value before the normalized color values are calculated. In brighter conditions, you should
   * use a smaller gain than in dark conditions. If your gain is too high, all of the colors will
   * report at or near 1, and you won't be able to determine what color you are actually looking at.
   * For this reason, it's better to err on the side of a lower gain (but always >=1).
   */
  private final float gain;

  private NormalizedRGBA colorSensorData;

  /**
   * Array containing HSV values. First element (0) contains hue Second element (1) contains
   * saturation Third element (2) contains value See <a
   * href="http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html">...</a>
   * for an explanation of HSV color.
   */
  private final float[] hsv = new float[3];

  public enum COLOR {
    BLUE,
    RED,
    YELLOW
  }

  // Hue is the fraction of the distance around the color wheel (degree / 360)
  //    public final static float[] BLUE_MIN_HSV = new float[]{208.0f/360.0f, 0.25f, 0.20f};
  //    public final static float[] BLUE_MAX_HSV = new float[]{240.0f/360.0f, 0.95f, 1.00f};

  public static final float[] BLUE_MIN_HSV =
      new float[] {205.0f, 0.2f, 0.0f}; // Blue samples read 240
  public static final float[] BLUE_MAX_HSV = new float[] {250.0f, 1.0f, 1.0f};

  //    public final static float[] RED1_MIN_HSV = new float[]{1.0f/360.0f, 0.25f, 0.20f};
  //    public final static float[] RED1_MAX_HSV = new float[]{10.0f/360.0f, 0.95f, 1.00f};

  public static final float[] RED1_MIN_HSV = new float[] {0.0f, 0.2f, 0.0f}; // Red samples read 0
  public static final float[] RED1_MAX_HSV = new float[] {40.0f, 1.0f, 1.0f};

  public static final float[] RED2_MIN_HSV = new float[] {350.0f, 0.2f, 0.2f};
  public static final float[] RED2_MAX_HSV = new float[] {360.0f, 1.0f, 1.0f};

  //    public final static float[] YELLOW_MIN_HSV = new float[]{60.0f/360.0f, 0.25f, 0.20f};
  //    public final static float[] YELLOW_MAX_HSV = new float[]{65.0f/360.0f, 1.0f, 1.00f};

  public static final float[] YELLOW_MIN_HSV =
      new float[] {55.0f, 0.2f, 0.0f}; // Yellow samples read 60
  public static final float[] YELLOW_MAX_HSV = new float[] {80.0f, 1.0f, 1.0f};

  /**
   * Constructor for the color sensor wrapper. Defaults to a gain of 2.
   *
   * @param hwMap Hardware map
   * @param sensorName Name of the color sensor as specified in the Robot Controller setup.
   */
  public ColorSensorWrapper(HardwareMap hwMap, String sensorName) {
    super(hwMap, RevColorSensorV3.class, sensorName);

    gain = 10.0f;
    sensor.setGain(gain);

    Msg.log(
        getClass().getSimpleName(), "ColorSensorWrapper", "Creating color sensor " + sensorName);
  }

  @Override
  public void reset() {
    sensor.setGain(gain);
  }

  /**
   * Get the current gain value
   *
   * @return Current color sensor gain
   */
  public float getGain() {
    return gain;
  }

  /**
   * Set the gain for the color sensor.
   *
   * @param gain Gain should be >= 1 because raw color sensor values may be too low and not use the
   *     0 - 1 range that is available.
   */
  public void setGain(float gain) {
    sensor.setGain(gain);
  }

  /**
   * This should be called to refresh the detected color. This can be called within an OpMode loop
   * to update the detection.
   */
  public void refreshColorSensorDetection() {
    colorSensorData = sensor.getNormalizedColors(); // Refresh the color sensor detection
    Color.colorToHSV(
        colorSensorData.toColor(),
        hsv); // Convert the normalized color info from sensor to HSV color
  }

  /**
   * Get normalized RGBA color of most recent detection. This does not refresh the detection.
   *
   * @return Normalized RGBA.
   */
  public NormalizedRGBA getDetectedRGBA() {
    return colorSensorData;
  }

  /**
   * Get HSV color of most recent detection. This does not refresh the detection.
   *
   * @return Detected HSV values as an array of floats.
   */
  public float[] getDetectedHSV() {
    return hsv;
  }

  public boolean isColorDetected(COLOR color) {
    float hue = hsv[0];
    float saturation = hsv[1];
    float value = hsv[2];

    switch (color) {
      case BLUE -> {
        return checkIfColor(hsv, BLUE_MIN_HSV, BLUE_MAX_HSV);
      }
      case RED -> {
        return (checkIfColor(hsv, RED1_MIN_HSV, RED1_MAX_HSV)
            || checkIfColor(hsv, RED2_MIN_HSV, RED2_MAX_HSV));
      }
      case YELLOW -> {
        return checkIfColor(hsv, YELLOW_MIN_HSV, YELLOW_MAX_HSV);
      }
    }
    return false;
  }

  private boolean checkIfColor(float[] hsv, float[] min_hsv, float[] max_hsv) {
    if ((hsv[0] >= min_hsv[0]) && (hsv[0] <= max_hsv[0])) {
      if ((hsv[1] >= min_hsv[1]) && (hsv[1] <= max_hsv[1])) {
        return (hsv[2] >= min_hsv[2]) && (hsv[2] <= max_hsv[2]);
      }
    }
    return false;
  }

  /** Read the distance from the sensor */
  public double readDistanceFromSensor(DistanceUnit unit) {
    lastDistanceSensorReading_mm = sensor.getDistance(unit);
    return lastDistanceSensorReading_mm;
  }

  /**
   * Get the distance from the sensor.
   *
   * @return Distance of detected object from sensor (mm)
   */
  public double getStoredDistance_mm() {
    return lastDistanceSensorReading_mm;
  }
}
