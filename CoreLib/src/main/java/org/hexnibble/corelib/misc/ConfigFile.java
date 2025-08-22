package org.hexnibble.corelib.misc;

import android.util.Log;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.Objects;
import java.util.Properties;
import org.hexnibble.corelib.exception.InvalidArgumentException;

/** Read and store data from config file Initial version written by Ben K. 2022-23 */
public class ConfigFile {
  private final String configFilePath = "/sdcard/FIRST/";
  private final String configFileName;
  private final Properties properties = new Properties();

  // Add each constant here and in initializeConstants()
  public static String CONFIG_FILE_VERSION;

  public static boolean DEBUG_MODE;

  public static float ROBOT_MIN_BATTERY_VOLTAGE_FOR_WARNING;

  // Drivetrain parameters
  public static double DRIVETRAIN_ECO_MODE_SPEED_MULTIPLICATION_FACTOR;
  public static double DRIVETRAIN_REGULAR_MODE_SPEED_MULTIPLICATION_FACTOR;
  public static double DRIVETRAIN_REGULAR_MODE_SPIN_MULTIPLICATION_FACTOR;

  // Drivetrain Odometry Wheel Locations
  /*    public static double DRIVETRAIN_FBENCODER_OFFSET_X;
      public static double DRIVETRAIN_FBENCODER_OFFSET_Y;
      public static double DRIVETRAIN_LRENCODER_OFFSET_X;
      public static double DRIVETRAIN_LRENCODER_OFFSET_Y;

      // Drivetrain Odometry Wheel Encoder Inversions
      public static boolean DRIVETRAIN_FBENCODER_INVERT_VALUES;
      public static boolean DRIVETRAIN_LRENCODER_INVERT_VALUES;
  */
  // Drivetrain Motion Profile Values
  public static double DRIVETRAIN_V_MAX;
  public static double DRIVETRAIN_A_MAX;
  public static double DRIVETRAIN_OMEGA_MAX;
  public static double DRIVETRAIN_ALPHA_MAX;

  // Drivetrain PID Values
  public static double DRIVETRAIN_TRANSLATION_PID_Ks;
  public static double DRIVETRAIN_TRANSLATION_PID_Kp;
  public static double DRIVETRAIN_TRANSLATION_PID_Ki;
  public static double DRIVETRAIN_TRANSLATION_PID_Kd;

  public static double DRIVETRAIN_Y_PID_Ks;
  public static double DRIVETRAIN_Y_PID_Kp;
  public static double DRIVETRAIN_Y_PID_Ki;
  public static double DRIVETRAIN_Y_PID_Kd;

  public static double DRIVETRAIN_ROTATION_PID_Ks;
  public static double DRIVETRAIN_ROTATION_PID_Kp;
  public static double DRIVETRAIN_ROTATION_PID_Ki;
  public static double DRIVETRAIN_ROTATION_PID_Kd;


  public static double SWERVE_PID_Kp;
  public static double SWERVE_PID_Ki;
  public static double SWERVE_PID_Kd;

  // Robot Dimensions
  //    public static double ROBOT_DIMENSIONS_LENGTH_MM;
  //    public static double ROBOT_DIMENSIONS_WIDTH_MM;
  /*
      // Robot Starting Position (X, Y should be mm in field CF; Heading should be IMU-style degrees, in field CF)
      public static double FIELD_START_POSITION_BLUE_LEFT_X_MM;
      public static double FIELD_START_POSITION_BLUE_LEFT_Y_MM;
      public static double FIELD_START_POSITION_BLUE_LEFT_HDG_DEGREES;
      public static double FIELD_START_POSITION_BLUE_RIGHT_X_MM;
      public static double FIELD_START_POSITION_BLUE_RIGHT_Y_MM;
      public static double FIELD_START_POSITION_BLUE_RIGHT_HDG_DEGREES;
      public static double FIELD_START_POSITION_RED_LEFT_X_MM;
      public static double FIELD_START_POSITION_RED_LEFT_Y_MM;
      public static double FIELD_START_POSITION_RED_LEFT_HDG_DEGREES;
      public static double FIELD_START_POSITION_RED_RIGHT_X_MM;
      public static double FIELD_START_POSITION_RED_RIGHT_Y_MM;
      public static double FIELD_START_POSITION_RED_RIGHT_HDG_DEGREES;
  */

  public ConfigFile(String configFileName) {
    this.configFileName = configFileName;
  }

  /** Read data from config file */
  public void readFromFile() {
    // Read data from config file "/sdcard/FIRST/Robot_Config.txt"
    ClassLoader classLoader = ClassLoader.getSystemClassLoader();
    URL resource =
        Objects.requireNonNull(
            classLoader.getResource(configFilePath + configFileName),
            configFilePath + configFileName + " not found.");
    Log.i(Constants.TAG, "ConfigFile.j: Reading config file " + configFilePath + configFileName);

    try {
      InputStream inputStream = new FileInputStream(resource.getFile());
      properties.load(inputStream);
      readConfigFile();
      inputStream.close();
    } catch (InvalidArgumentException iae) {
      String errorMessage =
          "ConfigFile.j: Invalid Parameter reading config file: " + iae.getMessage();
      Log.i(Constants.TAG, errorMessage);
      System.out.println(errorMessage);
      iae.printStackTrace();
      System.exit(0);
    } catch (IOException e) {
      String errorMessage = "ConfigFile.j: IO Error reading config file: " + e.getMessage();
      Log.i(Constants.TAG, errorMessage);
      System.out.println(errorMessage);
      e.printStackTrace();
    }
  }

  /**
   * Return Config Value as String
   *
   * @param propertyName Field Name
   * @return Value
   */
  public String getValueAsString(String propertyName) throws InvalidArgumentException {
    String value = properties.getProperty(propertyName);
    if (value == null) {
      throw new InvalidArgumentException("Unable to find property " + propertyName);
    } else {
      return value;
    }
  }

  /**
   * Return Config Value as Double
   *
   * @param propertyName Field Name
   * @return Value
   */
  public double getValueAsDouble(String propertyName) throws InvalidArgumentException {
    return Double.parseDouble(getValueAsString(propertyName));
  }

  /**
   * Return Config Value as Float
   *
   * @param propertyName Field Name
   * @return Value
   */
  public float getValueAsFloat(String propertyName) throws InvalidArgumentException {
    return Float.parseFloat(getValueAsString(propertyName));
  }

  /**
   * Return Config Value as Int
   *
   * @param propertyName Field Name
   * @return Value
   */
  public int getValueAsInt(String propertyName) throws InvalidArgumentException {
    return Integer.parseInt(getValueAsString(propertyName));
  }

  /**
   * Return Config Value as Long
   *
   * @param propertyName Field Name
   * @return Value
   */
  public long getValueAsLong(String propertyName) throws InvalidArgumentException {
    return Long.parseLong(getValueAsString(propertyName));
  }

  /**
   * Return Config Value as Boolean
   *
   * @param propertyName Field Name
   * @return Value
   */
  public boolean getValueAsBoolean(String propertyName) throws InvalidArgumentException {
    return Boolean.parseBoolean(getValueAsString(propertyName));
  }

  protected void readConfigFile() throws InvalidArgumentException {
    CONFIG_FILE_VERSION = getValueAsString("CONFIG_FILE_VERSION");
    Log.i(Constants.TAG, "ConfigFile.j: Reading config file version " + CONFIG_FILE_VERSION);
    /*
            // Control orientation
            Log.i(CoreLinearOpMode.TAG, "   Reading control hub orientation parameters.");
            String value = getValueAsString("ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION");
            switch (value) {
                case "UP" -> ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                case "DOWN" -> ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
                case "LEFT" -> ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
                case "RIGHT" -> ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
                case "FORWARD" -> ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
                case "BACKWARD" -> ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
                default -> throw new InvalidArgumentException("Invalid parameter for ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION: " + value);
            }

            value = getValueAsString("ROBOT_CONTROL_HUB_USB_FACING_DIRECTION");
            switch (value) {
                case "UP" -> ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;
                case "DOWN" -> ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
                case "LEFT" -> ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
                case "RIGHT" -> ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
                case "FORWARD" -> ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
                case "BACKWARD" -> ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
                default -> throw new InvalidArgumentException("Invalid parameter for ROBOT_CONTROL_HUB_USB_FACING_DIRECTION: " + value);
            }
    */
    // Controller stick parameters
    /*        Log.i(CoreLinearOpMode.TAG, "   Reading controller stick parameters.");
            CONTROLLER_STICK_DEAD_ZONE_X = getValueAsFloat("CONTROLLER_STICK_DEAD_ZONE_X");
            CONTROLLER_STICK_DEAD_ZONE_Y = getValueAsFloat("CONTROLLER_STICK_DEAD_ZONE_Y");
            CONTROLLER_STICK_MOVEMENT_THRESHOLD = getValueAsFloat("CONTROLLER_STICK_MOVEMENT_THRESHOLD");
            CONTROLLER_LEFT_TRIGGER_DEAD_ZONE = getValueAsFloat("CONTROLLER_LEFT_TRIGGER_DEAD_ZONE");
            CONTROLLER_RIGHT_TRIGGER_DEAD_ZONE = getValueAsFloat("CONTROLLER_RIGHT_TRIGGER_DEAD_ZONE");
    */
    DEBUG_MODE = getValueAsBoolean("DEBUG_MODE");
    Log.i(Constants.TAG, "   Debug Mode " + (DEBUG_MODE ? "ON" : "OFF"));

    ROBOT_MIN_BATTERY_VOLTAGE_FOR_WARNING =
        getValueAsFloat("ROBOT_MIN_BATTERY_VOLTAGE_FOR_WARNING");

    // Drivetrain parameters
    Log.i(Constants.TAG, "   Reading drivetrain parameters.");
    DRIVETRAIN_ECO_MODE_SPEED_MULTIPLICATION_FACTOR =
        getValueAsDouble("DRIVETRAIN_ECO_MODE_SPEED_MULTIPLICATION_FACTOR");
    DRIVETRAIN_REGULAR_MODE_SPEED_MULTIPLICATION_FACTOR =
        getValueAsDouble("DRIVETRAIN_REGULAR_MODE_SPEED_MULTIPLICATION_FACTOR");
    DRIVETRAIN_REGULAR_MODE_SPIN_MULTIPLICATION_FACTOR =
        getValueAsDouble("DRIVETRAIN_REGULAR_MODE_SPIN_MULTIPLICATION_FACTOR");
    /*
            DRIVETRAIN_FBENCODER_OFFSET_X = getValueAsDouble("DRIVETRAIN_FBENCODER_OFFSET_X");
            DRIVETRAIN_FBENCODER_OFFSET_Y = getValueAsDouble("DRIVETRAIN_FBENCODER_OFFSET_Y");
            DRIVETRAIN_LRENCODER_OFFSET_X = getValueAsDouble("DRIVETRAIN_LRENCODER_OFFSET_X");
            DRIVETRAIN_LRENCODER_OFFSET_Y = getValueAsDouble("DRIVETRAIN_LRENCODER_OFFSET_Y");

            // Drivetrain Odometry Wheel Encoder Inversions
            DRIVETRAIN_FBENCODER_INVERT_VALUES = getValueAsBoolean("DRIVETRAIN_FBENCODER_INVERT_VALUES");
            DRIVETRAIN_LRENCODER_INVERT_VALUES = getValueAsBoolean("DRIVETRAIN_LRENCODER_INVERT_VALUES");
    */
    DRIVETRAIN_V_MAX = getValueAsDouble("DRIVETRAIN_V_MAX");
    DRIVETRAIN_A_MAX = getValueAsDouble("DRIVETRAIN_A_MAX");
    DRIVETRAIN_OMEGA_MAX = getValueAsDouble("DRIVETRAIN_OMEGA_MAX");
    DRIVETRAIN_ALPHA_MAX = getValueAsDouble("DRIVETRAIN_ALPHA_MAX");

    DRIVETRAIN_TRANSLATION_PID_Ks = getValueAsDouble("DRIVETRAIN_TRANSLATION_PID_Ks");
    DRIVETRAIN_TRANSLATION_PID_Kp = getValueAsDouble("DRIVETRAIN_TRANSLATION_PID_Kp");
    DRIVETRAIN_TRANSLATION_PID_Ki = getValueAsDouble("DRIVETRAIN_TRANSLATION_PID_Ki");
    DRIVETRAIN_TRANSLATION_PID_Kd = getValueAsDouble("DRIVETRAIN_TRANSLATION_PID_Kd");

    DRIVETRAIN_Y_PID_Ks = getValueAsDouble("DRIVETRAIN_Y_PID_Ks");
    DRIVETRAIN_Y_PID_Kp = getValueAsDouble("DRIVETRAIN_Y_PID_Kp");
    DRIVETRAIN_Y_PID_Ki = getValueAsDouble("DRIVETRAIN_Y_PID_Ki");
    DRIVETRAIN_Y_PID_Kd = getValueAsDouble("DRIVETRAIN_Y_PID_Kd");

    DRIVETRAIN_ROTATION_PID_Ks = getValueAsDouble("DRIVETRAIN_ROTATION_PID_Ks");
    DRIVETRAIN_ROTATION_PID_Kp = getValueAsDouble("DRIVETRAIN_ROTATION_PID_Kp");
    DRIVETRAIN_ROTATION_PID_Ki = getValueAsDouble("DRIVETRAIN_ROTATION_PID_Ki");
    DRIVETRAIN_ROTATION_PID_Kd = getValueAsDouble("DRIVETRAIN_ROTATION_PID_Kd");

    SWERVE_PID_Kp = getValueAsDouble("SWERVE_PID_Kp");
    SWERVE_PID_Ki = getValueAsDouble("SWERVE_PID_Ki");
    SWERVE_PID_Kd = getValueAsDouble("SWERVE_PID_Kd");

    Log.i(Constants.TAG, "   Completed reading base ConfigFile parameters.");
  }
}
