package org.hexnibble.corelib.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
  public static final String TAG = "HexNibble";

  public static int MINIMUM_OP_MODE_LOOP_TIME_MS = 18;        // Minimum loop time to try and keep them more constant
  public static int LOOP_TIME_THRESHOLD_FOR_LOGGING_MS = 30;

  // region ** IMU Parameters **
  //    public static String IMU_NAME = "imu";
  //    // Control Hub Logo/USB Facing Direction Options are: UP, DOWN, LEFT, RIGHT, FORWARD,
  // BACKWARD
  //    public static RevHubOrientationOnRobot.LogoFacingDirection
  // ROBOT_CONTROL_HUB_LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
  //    public static RevHubOrientationOnRobot.UsbFacingDirection
  // ROBOT_CONTROL_HUB_USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
  // endregion ** IMU Parameters **

  // region /* Drivetrain Parameters */
  // Robot Dimensions
  public static final double ROBOT_DIMENSIONS_LENGTH_MM = 430.0; // Was 448.0 with Channel Fenders
  public static final double ROBOT_DIMENSIONS_WIDTH_MM = 379.35;

  // Odometry Wheel Locations
  // Offsets are relative to the center of rotation of the robot (origin), in mm
  // Positive X goes to right of robot
  // Positive Y goes to front of robot

  // GoBilda odometry pod omni wheels are 18mm wide (not the pod, the wheel)

  // IntoTheDeep: IntoTheDeep: The outside width between the two inner drivetrain walls is 245mm
  // The front and rear wheel axles are 312mm apart.

  // LR Encoder is on the right side of the robot (positive X), in front of the FB Encoder
  // X:   * 122.5mm from center to right drivetrain wall
  //      *  28.0mm from right drivetrain wall to center of omni wheel
  public static double DRIVETRAIN_LRENCODER_OFFSET_X = 150.5;
  public static double DRIVETRAIN_LRENCODER_OFFSET_Y = 34.0;
  public static DcMotor.Direction DRIVETRAIN_LRENCODER_RUN_DIRECTION =
      DcMotorSimple.Direction.FORWARD; // Moving to the right should yield POSITIVE X values

  // FB Encoder is on right side of robot (positive X)
  // X:   * 122.5mm from center to right drivetrain wall
  //      *  32.5mm from right drivetrain wall to omni wheel
  //      *   9.0mm for half the omni wheel width
  public static double DRIVETRAIN_FBENCODER_OFFSET_X = 164.0;
  public static double DRIVETRAIN_FBENCODER_OFFSET_Y = -20.0;
  public static DcMotor.Direction DRIVETRAIN_FBENCODER_RUN_DIRECTION =
      DcMotorSimple.Direction.FORWARD;
  // endregion

  // region ** Controller Dead Zones **
  public static float CONTROLLER_STICK_MOVEMENT_THRESHOLD = 0.02f;

  public static float CONTROLLER_STICK_DEAD_ZONE_X = 0.05f;
  public static float CONTROLLER_STICK_DEAD_ZONE_Y = 0.05f;
  public static float CONTROLLER_LEFT_TRIGGER_DEAD_ZONE = 0.05f;
  public static float CONTROLLER_RIGHT_TRIGGER_DEAD_ZONE = 0.05f;
  // endregion ** Controller Dead Zones **

  // Minimum threshold for motor power to exceed to send a new motor command (compared to the
  // previously sent value)
  public static double MOTOR_POWER_THRESHOLD_FOR_NEW_COMMAND = 0.01;


  public static boolean USE_ROAD_RUNNER = false;
  public static boolean USE_PEDRO_PATHING = false;

  public static boolean USE_FTCONTROL_DASHBOARD = false;

  /*
      public static void setConstants(Class<?> autonomousConstants) {
          autoConstants = autonomousConstants;
          setup();
      }

      private static void setup() {
          try {
              Class.forName(autoConstants.getName());
          }
          catch (ClassNotFoundException cnfe) {
              Msg.log("Constants", "setup", "Class not found exception " + cnfe.toString());
          }
      }
  */

  // region /* Robot starting positions */
  // Blue Alliance
  public static double FIELD_START_POSITION_BLUE_LEFT_X_MM;
  public static double FIELD_START_POSITION_BLUE_RIGHT_X_MM;
  public static double FIELD_START_POSITION_BLUE_LEFT_Y_MM;
  public static double FIELD_START_POSITION_BLUE_RIGHT_Y_MM;
  public static double FIELD_START_POSITION_BLUE_LEFT_HDG_DEGREES;
  public static double FIELD_START_POSITION_BLUE_RIGHT_HDG_DEGREES;

  // Red Alliance
  public static double FIELD_START_POSITION_RED_LEFT_X_MM;
  public static double FIELD_START_POSITION_RED_RIGHT_X_MM;
  public static double FIELD_START_POSITION_RED_LEFT_Y_MM;
  public static double FIELD_START_POSITION_RED_RIGHT_Y_MM;
  public static double FIELD_START_POSITION_RED_LEFT_HDG_DEGREES;
  public static double FIELD_START_POSITION_RED_RIGHT_HDG_DEGREES;
  // endregion
}
