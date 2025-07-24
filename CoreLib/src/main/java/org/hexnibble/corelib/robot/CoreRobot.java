package org.hexnibble.corelib.robot;

import static org.hexnibble.corelib.misc.Constants.TAG;

import android.graphics.Color;
import android.util.Log;
import androidx.annotation.ColorInt;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.hexnibble.corelib.commands.DrivetrainRC;
import org.hexnibble.corelib.commands.MultiDrivetrainRC;
import org.hexnibble.corelib.commands.RobotCommand;
import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.pid.PIDSettings;
import org.hexnibble.corelib.motion.Waypoint;
import org.hexnibble.corelib.robot_system.CoreRobotSystem;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.sensor.IMUWrapper;

// region ** Hub Ports **
/* ///////////////////////////
Encoder ports 0 and 3 are hardware (1 and 2 are software).

Rev Control Hub Ports and Names*************************

Motor 0: LFMotor                            Encoder 0: FBEncoder
Motor 1: LBMotor                            Encoder 1: -
Motor 2: -                                  Encoder 2: -
Motor 3: -                                  Encoder 3: -

Servo 0: -                                  Servo 1: -
Servo 2: -                                  Servo 3: -
Servo 4: -                                  Servo 5: -

// Digital ports are used for Rev Touch Sensor, Magnetic Limit Switch, and LED
// Even-numbered ports are n. Odd-numbered ports are n+1.
// Rev touch sensors are wired to N+1 channel so they must be configured on odd-numbered digital ports.
// Rev LEDs are wired with N = green, N+1 = red
Digital 0:  -                               Digital 1: -
Digital 2:  -                               Digital 3: -
Digital 4:  -                               Digital 5: -
Digital 6:  -                               Digital 7: -

// I2C ports are used for Rev Color Sensor and 2M Distance Sensor. Both use address 0x52.
I2C 0: --       IMU (0x28)
I2C 1: -
I2C 2: -
I2C 3: -

RS485: Expansion Hub

USB3: Webcam1

///////////////////////////
Rev Expansion Hub Ports and Names
Motor 0: RFMotor                            Encoder 0: LREncoder
Motor 1: RBMotor                            Encoder 1: --
Motor 2: --                                 Encoder 2: --
Motor 3: --                                 Encoder 3: --

Servo 0: -                                  Servo 1: -
Servo 2: -                                  Servo 3: -
Servo 4: -                                  Servo 5: -

// Digital ports are used for Rev Touch Sensor, Magnetic Limit Switch, and LED
// Even-numbered ports are n. Odd-numbered ports are n+1.
// Rev touch sensors are wired to N+1 channel so they must be configured on odd-numbered digital ports.
// Rev LEDs are wired with N = green, N+1 = red
Digital 0:  -                               Digital 1: -
Digital 2:  -                               Digital 3: -
Digital 4:  -                               Digital 5: -
Digital 6:  -                               Digital 7: -

// I2C ports are used for Rev Color Sensor and 2M Distance Sensor. Both use address 0x52.
I2C 0: -
I2C 1: -
I2C 2: HSlideColorSensor
I2C 3: -
*/
// endregion ** Hub Ports **

public class CoreRobot extends CoreRobotSystem {
  // region ** Hub Parameters **
  protected String CONTROL_HUB_NAME = "Control Hub";
  protected String EXPANSION_HUB_NAME = "Expansion Hub 2";
  protected String SERVO_HUB_NAME = "Servo Hub 3";

  // Control Hub Logo/USB Facing Direction Options are: UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
  protected RevHubOrientationOnRobot.LogoFacingDirection CH_LOGO_FACING_DIRECTION =
      RevHubOrientationOnRobot.LogoFacingDirection.UP;
  protected RevHubOrientationOnRobot.UsbFacingDirection CH_USB_FACING_DIRECTION =
      RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

  public enum HUB_TYPE {
    CONTROL_HUB,
    EXPANSION_HUB
  }

  private LynxModule controlHub; // For bulk reads from control/expansion hubs
  private LynxModule expansionHub; // For bulk reads from control/expansion hubs
  private LynxModule servoHub;
  private long currentCHubBulkReadTime;
  private long previousCHubBulkReadTime;
  private long currentEHubBulkReadTime;
  private long previousEHubBulkReadTime;
  // endregion ** Hub Parameters **

  // region ** IMU Parameters **
  // IMUs shipped prior to Sept 2022 were Bosch BNO055. The Rev external 9-axis IMU also use the
  // BNO055. The external IMU I2C port can be used as the equivalent of the control hub USB port
  // when defining orientation.
  // Beginning Sept 2022, Bosch BHI260AP were used.
  protected String IMU_NAME = "imu";
  protected IMUWrapper IMU;
//  protected IMUIface IMU;
  // endregion ** IMU Parameters **

  // Drivetrain
  public MecanumDrivetrain drivetrain;
  protected boolean
      fieldCentricDrive; // Flag whether to offset heading when sending driving commands to perform
                         // field- vs. robot-centric driving

  protected final int TARGET_POSITION_TOLERANCE = 5;

  // Odometry
  protected BaseOdometry odometry;

  // Motors and Servos
  public enum MOTOR_MECHANISM_TYPE {
    LINEAR,
    LEAD_SCREW,
    ROTATION,
    DEFAULT
  }

  protected Map<String, CoreRobotSystem> robotSystemList;

  // Sensors
  protected Limelight3A limelight;
  protected SparkFunLEDStick LEDStick;

  // Vision
  public VisionPortal visionPortal;

  // Commands
  protected final List<RobotCommand> robotDrivetrainCommandQueue = new ArrayList<>();
  protected final List<String> completedRCList =
      new ArrayList<>(); // Store IDs of completed Robot Commands (we do not save cancelled
                         // commands)
  protected boolean cancelCurrentDrivetrainRobotCommand;

  protected boolean isPTOEngaged;

  // Logging
  protected final String className = getClass().getSimpleName();

  /**
   * CoreRobot constructor. This will set up the hardware map, IMU, hub bulk reads, and call
   * initializeRobot().
   *
   * @param hwMap hardware Map
   */
  public CoreRobot(@NonNull HardwareMap hwMap, String robotName) {
    super(hwMap, robotName);
    this.cancelCurrentDrivetrainRobotCommand = false;
    this.isPTOEngaged = false;

    this.fieldCentricDrive = true;

    this.robotSystemList = new HashMap<>();

    identifyHubs(hwMap);
    setHubsBulkCachingModeToManual();

    initializeSystem();
  }

  /**
   * Override this method to initialize robot components (e.g. drivetrain, servos, sensors, etc) on
   * robot object creation. It is always called from the base constructor. So be careful only to use
   * constants. Instance variables will not have been initialized yet.
   */
  @Override
  public void initializeSystem() {
    super.initializeSystem();

    IMU = new IMUWrapper(hwMap, IMU_NAME, CH_LOGO_FACING_DIRECTION, CH_USB_FACING_DIRECTION);

    // Create drivetrain object
    Msg.logIfDebug(className, "initializeRobot", "Creating mecanum drivetrain object.");
    drivetrain = new MecanumDrivetrain(hwMap,
            "LFMotor", DcMotor.Direction.REVERSE,
            BaseMotorWrapper.ENCODER.GO_BILDA_ODOPOD, DcMotorSimple.Direction.FORWARD,
            "RFMotor", DcMotor.Direction.FORWARD,
            BaseMotorWrapper.ENCODER.GO_BILDA_ODOPOD, DcMotorSimple.Direction.FORWARD,
            "LBMotor", DcMotor.Direction.REVERSE,
            BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,
            "RBMotor", DcMotor.Direction.FORWARD,
            BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
            BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_435, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
            1.0, TARGET_POSITION_TOLERANCE, 48.0);

    addRobotSystemToList("Drivetrain", drivetrain);

    // Create odometry object
    Msg.logIfDebug(className, "initializeRobot", "Creating odometry servos and object.");
    odometry =
        new TwoWheelOdometry(
            Arrays.asList(
                new Pose2D(
                    Constants.DRIVETRAIN_LRENCODER_OFFSET_X,
                    Constants.DRIVETRAIN_LRENCODER_OFFSET_Y,
                    Math.PI / 2.0), // Location of LR odometry wheel - x movements
                new Pose2D(
                    Constants.DRIVETRAIN_FBENCODER_OFFSET_X,
                    Constants.DRIVETRAIN_FBENCODER_OFFSET_Y,
                    0.0) // Location of FB odometry wheel - y movements
                ),
            Arrays.asList(
                drivetrain.getWheelMotorObject(
                    MecanumDrivetrain.WHEEL_MOTOR_NAME.LF), // LR odometry wheel (x movements)
                drivetrain.getWheelMotorObject(
                    MecanumDrivetrain.WHEEL_MOTOR_NAME.RF) // FB odometry wheel (y movements)
                ));

    robotSystemList.values().forEach(CoreRobotSystem::initializeSystem);
  }

  /**
   * This method is called when creating a new OpMode if the robot object already exists. It can be
   * used to reset variables.
   */
  @Override
  public void resetSystem() {
    super.resetSystem();
    Msg.log(className, "resetRobot", "Resetting robot.");

    identifyHubs(hwMap);
    setHubsBulkCachingModeToManual();

    isPTOEngaged = false;

    // Clear robot command-related lists
    robotDrivetrainCommandQueue.clear();
    completedRCList.clear();

    cancelCurrentDrivetrainRobotCommand = false;

    if (drivetrain != null) {
      drivetrain.resetSystem();
//      drivetrainManualMovementUpdated = false;
    }

//    drivetrainManualMovement_X = 0.0;
//    drivetrainManualMovement_Y = 0.0;
//    drivetrainManualMovement_Spin = 0.0;

    if (limelight != null) {
      limelight.stop();
    }

    if (LEDStick != null) {
      LEDStick.setBrightness(4);
      LEDStick.setColor(0);
    }

//    //        motorList.values().forEach(BaseMotorWrapper::reset);
//    servoList.values().forEach(BaseServoWrapper::reset);
//    sensorList.values().forEach(CoreSensorWrapper::reset);
//
    robotSystemList.values().forEach(CoreRobotSystem::resetSystem);
  }

  public void destructor() {
//    drivetrain.onCloseJNI();
  }

  /**
   * Obtain voltage reading from main robot battery.
   *
   * @return Battery voltage (V)
   */
  public double getRobotBatteryVoltage() {
    return controlHub.getInputVoltage(VoltageUnit.VOLTS);
  }

  // region ** Hub Functions **
  protected void identifyHubs(HardwareMap hwMap) {
    controlHub = hwMap.get(LynxModule.class, CONTROL_HUB_NAME);

    try {
      expansionHub = hwMap.get(LynxModule.class, EXPANSION_HUB_NAME);
    }
    catch (IllegalArgumentException iae) {
      Msg.log(className, "identifyHubs", EXPANSION_HUB_NAME + " not found.");
    }

    try {
      servoHub = hwMap.get(LynxModule.class, SERVO_HUB_NAME);
    }
    catch (IllegalArgumentException iae) {
      Msg.log(className, "identifyHubs", SERVO_HUB_NAME + " not found.");
    }
  }

  /** Bulk read the control hub. */
  public void bulkReadControlHub() {
    previousCHubBulkReadTime = currentCHubBulkReadTime;
    currentCHubBulkReadTime = System.currentTimeMillis();
    controlHub.clearBulkCache();
//    Msg.log("CHub bulk read time=" + (System.currentTimeMillis() - currentCHubBulkReadTime));
  }

  /** Bulk read the expansion hub if it exists. */
  public void bulkReadExpansionHub() {
    if (expansionHub != null) {
      previousEHubBulkReadTime = currentEHubBulkReadTime;
      currentEHubBulkReadTime = System.currentTimeMillis();
      expansionHub.clearBulkCache();
    }
  }

  public LynxModule.BulkCachingMode getHubBulkCachingMode(CoreRobot.HUB_TYPE hubType) {
    LynxModule hub =
        switch (hubType) {
          case CONTROL_HUB -> controlHub;
          case EXPANSION_HUB -> expansionHub;
        };

    if (hub != null) {
      return hub.getBulkCachingMode();
    } else return null;
  }

  public void setHubsBulkCachingModeToManual() {
    // Set bulk caching mode for hubs.
    // MANUAL provides the most control. But the cache must be refreshed manually
    // AUTO will perform a new bulk read when any particular hardware request is repeated
    // OFF turns off bulk reads
    controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    if (expansionHub != null) {
      expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
  }

  public int getCHubBulkReadTimeDelta() {
    return (int) (currentCHubBulkReadTime - previousCHubBulkReadTime);
  }

  public int getEHubBulkReadTimeDelta() {
    return (int) (currentEHubBulkReadTime - previousEHubBulkReadTime);
  }

  // endregion ** Hub Functions **

  // region ** IMU Functions **
  /** Reset the IMU heading (yaw). */
  public void resetIMUHeading() {
    IMU.resetIMUHeading();
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
    return IMU.readIMUHeading();
  }

  public double getStoredIMUHeadingDegrees() {
    return IMU.getStoredIMUHeadingDegrees();
  }

  // endregion IMU Functions

  // region ** Odometry Functions **

  /**
   * Retrieve a list of odometry encoder positions (in mm).
   *
   * @return ArrayList of encoder positions (in mm)
   */
  public ArrayList<Double> getOdometryEncoderPositions_mm() {
    if (odometry != null) {
      return odometry.getEncoderPositionList_mm();
    } else {
      return null;
    }
  }

  /**
   * Get current pose estimate in alliance CF.
   *
   * @return Pose estimate (alliance CF), with angle in radians.
   */
  public Pose2D getRobotPoseEstimate() {
    return odometry.getPoseEstimate();
  }

  /**
   * Retrieve the last time the encoder positions were read (system ms)
   *
   * @return Last encoder read time, in ms
   */
  public long getLastPoseEstimateTime() {
    return odometry.getLastEncoderReadTime_ms();
  }

  /**
   * Set the current estimated alliance-centric pose.
   *
   * @param newPose New pose (alliance-centric)
   */
  public void setPoseEstimate(Pose2D newPose) {
    if (odometry != null) {
      odometry.setPoseEstimate(newPose);
    }
  }

  public void resetOdometryEncoders() {
    if (odometry != null) {
      odometry.resetEncoders();
    }
  }

  // endregion ** Odometry Functions **

  // region ** Motor Functions **
  //    protected BaseMotorWrapper addMotor(String motorName, BaseMotorWrapper.MOTOR_MODEL model,
  // DcMotor.Direction runDirection, DcMotor.RunMode runMode, double externalGearReduction,
  //                                        BaseMotorWrapper.ENCODER encoder, DcMotor.Direction
  // encoderDirection) {
  //        Log.i(TAG, "Creating motor " + motorName);
  //        assert (motorList != null);
  //        BaseMotorWrapper motor = new BaseMotorWrapper(hwMap, motorName, model, runDirection,
  // runMode, encoder, encoderDirection, externalGearReduction);
  //        motorList.put(motorName, motor);
  //        return motor;
  //    }
  //
  //    protected LinearMotor addLinearMotor(String motorName, BaseMotorWrapper.MOTOR_MODEL model,
  // DcMotor.Direction runDirection, DcMotor.RunMode runMode, BaseMotorWrapper.ENCODER encoder,
  // DcMotor.Direction encoderDirection, double externalGearReduction,
  //                                         double outputDiameter_mm, double minPosition_mm, double
  // maxPosition_mm) {
  //        Log.i(TAG, "Creating linear motor " + motorName);
  //        assert (motorList != null);
  //        LinearMotor motor = new LinearMotor(hwMap, motorName, model, runDirection, runMode,
  // encoder, encoderDirection, externalGearReduction, outputDiameter_mm, minPosition_mm,
  // maxPosition_mm, 1);
  //        motorList.put(motorName, motor);
  //        return motor;
  //    }
  //
  //    protected RotationMotor addRotationMotor(String motorName, BaseMotorWrapper.MOTOR_MODEL
  // model, DcMotor.Direction runDirection, DcMotor.RunMode runMode, BaseMotorWrapper.ENCODER
  // encoder, DcMotor.Direction encoderDirection, double externalGearReduction,
  //                                             double minPositionDegrees, double
  // maxPositionDegrees) {
  //        Log.i(TAG, "Creating rotation motor " + motorName);
  //        assert (motorList != null);
  //        RotationMotor motor = new RotationMotor(hwMap, motorName, model, runDirection, runMode,
  // encoder, encoderDirection, externalGearReduction, minPositionDegrees, maxPositionDegrees, 1.0);
  //        motorList.put(motorName, motor);
  //        return motor;
  //    }
  //
  //    protected LeadScrewMotor addLeadScrewMotor(String motorName, BaseMotorWrapper.MOTOR_MODEL
  // model, DcMotor.Direction runDirection, DcMotor.RunMode runMode, BaseMotorWrapper.ENCODER
  // encoder, DcMotor.Direction encoderDirection, double externalGearReduction,
  //                                               double distancePerRotation_mm, double
  // minPosition_mm, double maxPosition_mm) {
  //        Log.i(TAG, "Creating lead screw motor " + motorName);
  //        assert (motorList != null);
  //        LeadScrewMotor motor = new LeadScrewMotor(hwMap, motorName, model, runDirection,
  // runMode, encoder, encoderDirection, externalGearReduction, distancePerRotation_mm,
  // minPosition_mm, maxPosition_mm);
  //        motorList.put(motorName, motor);
  //        return motor;
  //    }

  //    public MOTOR_MECHANISM_TYPE getMotorMechanismType(String motorName) {
  //        assert (motorList != null);
  //        BaseMotorWrapper motor = motorList.get(motorName);
  //        assert (motor != null);
  //        if (motor instanceof LinearMotor) {
  //            return MOTOR_MECHANISM_TYPE.LINEAR;
  //        } else if (motor instanceof LeadScrewMotor) {
  //            return MOTOR_MECHANISM_TYPE.LEAD_SCREW;
  //        } else if (motor instanceof RotationMotor) {
  //            return MOTOR_MECHANISM_TYPE.ROTATION;
  //        } else return MOTOR_MECHANISM_TYPE.DEFAULT;
  //    }

  //    public String[] getMotorNames() {
  //        Set<String> listOfMotorNames = motorList.keySet();
  //        return listOfMotorNames.toArray(new String[0]);
  //    }

  public void setDrivetrainBrakeMode(DcMotor.ZeroPowerBehavior brakeMode) {
    drivetrain.setBrakeMode(brakeMode);
  }

  public double getDrivetrainMotorCurrent(MecanumDrivetrain.WHEEL_MOTOR_NAME motorName) {
    return drivetrain.getMotorCurrent(motorName);
  }

  public double getDrivetrainMotorPower(MecanumDrivetrain.WHEEL_MOTOR_NAME motorName) {
    return drivetrain.getMotorPower(motorName);
  }

  /*
      public double getMotorCurrentVelocityRPM(String motorName) {
          assert(motorList != null);
          return motorList.get(motorName).getCurrentVelocityRPM();
      }
  */
  //    public void setMotorPower(String motorName, double power) {
  //        assert (motorList != null);
  //        assert (motorList.get(motorName) != null);
  //        motorList.get(motorName).setPower(power);
  //    }
  //
  //    public void moveMotorToPosition_mm(String motorName, double motorPower, double
  // targetPosition_mm) {
  //        assert (motorList != null);
  //        assert (motorList.get(motorName) instanceof LinearMotor);
  //        ((LinearMotor) motorList.get(motorName)).moveToPosition_mm(targetPosition_mm,
  // motorPower, true);
  //    }

  //
  //    /**
  //     * Get the current motor position.
  //     *
  //     * @param motorName String name of the motor
  //     * @return Current position (mm for linear motor, degrees for rotation motor, or counts for
  // other)
  //     */
  //    public double getMotorPosition(String motorName) {
  //        assert (motorList != null);
  //
  //        BaseMotorWrapper motor = motorList.get(motorName);
  //        assert (motor != null);
  //
  //        if (motor instanceof LinearMotor) {
  //            return ((LinearMotor) motor).getCurrentPosition_mm();
  //        } else if (motor instanceof LeadScrewMotor) {
  //            return ((LeadScrewMotor) motor).getCurrentPosition_mm();
  //        } else if (motor instanceof RotationMotor) {
  //            return ((RotationMotor) motor).getCurrentPositionDegrees();
  //        } else {
  //            return motor.getCurrentPosition();
  //        }
  //    }

  //    /**
  //     * Move the motor to the minimum specified position.
  //     * This is intended to be used with LinearMotor and RotationMotor (it will not work for
  // others).
  //     *
  //     * @param motorName  String name of the motor
  //     * @param motorPower Motor power (0 - 1) for movement
  //     */
  //    public void moveMotorToMinPosition(String motorName, double motorPower) {
  //        assert (motorList != null);
  //
  //        BaseMotorWrapper motor = motorList.get(motorName);
  //        assert (motor != null);
  //
  //        if (motor instanceof LinearMotor) {
  //            ((LinearMotor) motor).moveToMinPosition(motorPower);
  //        } else if (motor instanceof LeadScrewMotor) {
  //            ((LeadScrewMotor) motor).moveToMinPosition(motorPower);
  //        } else if (motor instanceof RotationMotor) {
  //            ((RotationMotor) motor).moveToMinAngle(motorPower);
  //        }
  //    }

  //    /**
  //     * Move the motor to the maximum specified position.
  //     * This is intended to be used with LinearMotor and RotationMotor (it will not work for
  // others).
  //     *
  //     * @param motorName  String name of the motor
  //     * @param motorPower Motor power (0 - 1) for movement
  //     */
  //    public void moveMotorToMaxPosition(String motorName, double motorPower) {
  //        assert (motorList != null);
  //
  //        BaseMotorWrapper motor = motorList.get(motorName);
  //        assert (motor != null);
  //
  //        if (motor instanceof LinearMotor) {
  //            ((LinearMotor) motor).moveToMaxPosition(motorPower);
  //        } else if (motor instanceof LeadScrewMotor) {
  //            ((LeadScrewMotor) motor).moveToMaxPosition(motorPower);
  //        } else if (motor instanceof RotationMotor) {
  //            ((RotationMotor) motor).moveToMaxAngle(motorPower);
  //        }
  //    }

  //    /**
  //     * Stop the motor at its current position. If power is left as is, then the motor will
  // remain running to try and maintain the position.
  //     * This is intended to be used with RotationMotor.
  //     *
  //     * @param motorName      String name of the motor
  //     * @param setPowerToZero If true, will set the Power to 0.
  //     */
  //    public void stopMotorAtCurrentPosition(String motorName, boolean setPowerToZero) {
  //        assert (motorList != null);
  //
  //        BaseMotorWrapper motor = motorList.get(motorName);
  //        assert (motor != null);
  //
  //        if (motor instanceof RotationMotor) {
  //            ((RotationMotor) motor).stopMotor(setPowerToZero);
  //        }
  //    }
  // endregion ** Motor Functions **


  protected void addRobotSystemToList(String systemName, CoreRobotSystem robotSystem) {
    robotSystemList.put(systemName, robotSystem);
  }

  /**
   * Clear system robot command list for a robot system. This cancels any auto movements involving that
   * system.
   *
   * @param systemName Name of the system for the manual movement
   */
  public void clearSystemRCList(String systemName) {
    robotSystemList.get(systemName).clearSystemRCList();
  }
//
//  public double getLinearSystemCurrentPosition_mm(
//      String complexSystemName, String linearSystemName) {
//    return robotSystemList
//        .get(complexSystemName)
//        .getLinearMechanismCurrentPosition_mm(linearSystemName);
//  }
//
//
//  public void resetLinearSystemMotorEncoder(String complexSystemName, String linearSystemName) {
//    robotSystemList.get(complexSystemName).resetLinearMechanismEncoder(linearSystemName);
//  }

  public void setLinearSystemMotorRunMode(
      String complexSystemName, String linearSystemName, DcMotor.RunMode runMode) {
    robotSystemList
        .get(complexSystemName)
        .setLinearMechanismMotorRunMode(linearSystemName, runMode);
  }

  public void setLinearSystemMotorPower(
      String complexSystemName, String linearSystemName, double motorPower) {
    robotSystemList
        .get(complexSystemName)
        .setLinearMechanismMotorPower(linearSystemName, motorPower);
  }

  // region ** Distance Sensor Functions **
  //    public DistanceSensorWrapper addDistanceSensor(String sensorName) {
  //        DistanceSensorWrapper sensor = new DistanceSensorWrapper(hwMap, sensorName);
  //        if (sensorList == null) {
  //            sensorList = new HashMap<>();
  //        }
  //        sensorList.put(sensorName, sensor);
  //        return sensor;
  //    }

  //    /**
  //     * Returns the distance sensor reading (mm)
  //     * @param sensorName Name of the distance sensor as defined in the robot configuration.
  //     * @return Distance (mm) reading
  //     */
  //    public double getDistanceSensorDistance(String sensorName) {
  //        // Check for null object because if there is a hardware malfunction, the sensor may not
  // have been detected and added to the list.
  //        // We do not want the robot to stop working because of this.
  //        DistanceSensorWrapper sensor = (DistanceSensorWrapper) sensorList.get(sensorName);
  //        if (sensor != null) {
  //            return sensor.getDistanceSensorReading();
  //        }
  //        else return -1.0;
  //    }

  // endregion


  // region ** LED Functions **
  /**
   * Set the color of the entire LED strip. 0 = Off
   *
   * @param color
   */
  public void setLEDStickColor(@ColorInt int color) {
    if (LEDStick != null) {
      LEDStick.setColor(color);
    }
  }

  /**
   * Set the color of a single LED on the strip
   *
   * @param position Integer position of the LED on the strip (range 1-10)
   * @param color
   */
  public void setSingleLEDColor(int position, @ColorInt int color) {
    if (LEDStick != null) {
      int rangeCheckedPosition;
      rangeCheckedPosition = Math.max(position, 1);
      rangeCheckedPosition = Math.min(rangeCheckedPosition, 10);

      Log.i(TAG, "Setting LED position " + position + " to " + color);
      LEDStick.setColor(rangeCheckedPosition, color);
    }
  }

  public void setMultiLEDColor(int[] positions, @ColorInt int color) {
    for (int pos : positions) {
      setSingleLEDColor(pos, color);
    }
  }

  public void setMultiLEDColor(int[] positions, String color) {
    for (int pos : positions) {
      setSingleLEDColor(pos, Color.parseColor(color));
    }
  }

  public void turnOffMultiLEDColor(int[] positions) {
    for (int pos : positions) {
      setSingleLEDColor(pos, 0);
    }
  }

  /**
   * Set LED brightness
   *
   * @param brightness Brightness value from 0-31
   */
  public void setLEDStickBrightness(int brightness) {
    if (LEDStick != null) {
      LEDStick.setBrightness(brightness);
    }
  }

  // endregion **

  // region /* Limelight */
  /**
   * Set pipeline: 0 = Red Sample 1 = Blue Sample 2 = Yellow Sample
   *
   * @param pipelineIndex
   */
  public void setLimelightPipeline(int pipelineIndex) {
    Msg.log(className, "setLimelightPipeline", "Switching LL pipeline to " + pipelineIndex);
    limelight.pipelineSwitch(pipelineIndex);
  }

  /** Start polling of Limelight data */
  public void startLimeLight() {
    if (limelight != null) {
      limelight.start();
    }
  }

  public void stopLimeLight() {
    if (limelight != null) {
      limelight.stop();
    }
  }

  public void closeLimeLight() {
    if (limelight != null) {
      limelight.close();
    }
  }

  public boolean isLimelightBeingUsed() {
    return !(limelight == null);
  }

  public boolean isLimelightDisconnected() {
    return limelight == null || !limelight.isConnected();
  }

  public boolean isLimeLightRunning() {
    return limelight != null && limelight.isRunning();
  }
  // endregion

  // region ** Robot Command Functions **
  /**
   * Check if the specified command ID is complete. This is done by searching the list of completed
   * commands to see if the specified command ID is present. If the ID is found, it is removed from
   * the list.
   *
   * @param commandID
   * @return True/False whether the command ID was present
   */
  public boolean isCommandComplete(String commandID) {
    if (completedRCList.contains(commandID)) {
      completedRCList.remove(commandID);
      return true;
    } else return false;
  }

  /**
   * Queue a movement to X,Y coordinates, with an IMU-style heading, in alliance CF.
   *
   * @param commandID String label for this command
   * @param allianceCFPose Alliance CF X,Y (mm) location and IMU-Style orientation in radians (+
   *     values are CCW)
   * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
   */
  public void qAbsoluteDrivetrainRC(
      String commandID, Pose2D allianceCFPose, DrivetrainRC.ROTATION_DIRECTION rotationDirection) {
    /*        DrivetrainRC command = new DrivetrainRC(
                    commandID, DrivetrainRC.COMMAND_TYPE.ABSOLUTE, DrivetrainRC.COMMAND_CONTENTS.TRANSLATION_AND_ROTATION,
                    allianceCFPose,
                    rotationDirection, odometry, -1);
    */
    Waypoint waypoint = new Waypoint(allianceCFPose);
    List<Waypoint> waypointList = new ArrayList<Waypoint>();
    waypointList.add(waypoint);

    MultiDrivetrainRC command =
        new MultiDrivetrainRC(
            commandID,
            MultiDrivetrainRC.COMMAND_TYPE.ABSOLUTE,
            MultiDrivetrainRC.COMMAND_CONTENTS.TRANSLATION_AND_ROTATION,
            waypointList,
            rotationDirection,
            odometry,
            1.0,
            -1);

    command.initializeCommand();
    robotDrivetrainCommandQueue.add(command);
  }

  /**
   * Queue a *list of waypoint* movements to X,Y coordinates, with an IMU-style heading, in alliance
   * CF.
   *
   * @param commandID String label for this command
   * @param waypointList Alliance CF X,Y (mm) location and IMU-Style orientation in radians (+
   *     values are CCW)
   * @param rotationDirection CLOCKWISE or COUNTERCLOCKWISE
   * @param maxVelocityAdjustment Maximum velocity/acceleration adjustment factor (0 - 1.0)
   * @param maxCommandDuration maximum duration in ms
   */
  public void qAbsoluteDrivetrainRC(
      String commandID,
      List<Waypoint> waypointList,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      double maxVelocityAdjustment,
      int maxCommandDuration) {

    Log.i(TAG, "CoreRobot: qAbsoluteDrivetrainRC");

    MultiDrivetrainRC command =
        new MultiDrivetrainRC(
            commandID,
            MultiDrivetrainRC.COMMAND_TYPE.ABSOLUTE,
            MultiDrivetrainRC.COMMAND_CONTENTS.TRANSLATION_AND_ROTATION,
            waypointList,
            rotationDirection,
            odometry,
            maxVelocityAdjustment,
            maxCommandDuration);

    command.initializeCommand();
    robotDrivetrainCommandQueue.add(command);
  }

  /**
   * This is only used for PID tuning opmode. Queue a delta translation with X,Y, and IMU-style
   * heading, in alliance CF. These deltas will be relative to the location at the time the command
   * is initialized.
   *
   * @param commandID String label for this command
   * @param allianceCFDeltaPose
   * @param rotationDirection
   * @param xyPIDSettings PID settings for translation movements
   * @param rotationPIDSettings PID settings for rotation movements
   */
  public void qDeltaDrivetrainRC(
      String commandID,
      Pose2D allianceCFDeltaPose,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      PIDSettings xyPIDSettings,
      PIDSettings rotationPIDSettings) {
    Pose2D targetAllianceCFPose = new Pose2D(getRobotPoseEstimate());
    targetAllianceCFPose.x += allianceCFDeltaPose.x;
    targetAllianceCFPose.y += allianceCFDeltaPose.y;
    targetAllianceCFPose.heading =
        Field.enforceIMUHeadingRangeRadians(
            targetAllianceCFPose.heading + allianceCFDeltaPose.heading);
    Log.i(
        TAG,
        "CoreRobot.j: qDeltaDrivetrainRC() - New target AllianceCF (x,y,hdg degs): "
            + targetAllianceCFPose.x
            + ", "
            + targetAllianceCFPose.y
            + ", "
            + Math.toDegrees(targetAllianceCFPose.heading));
    Waypoint waypoint = new Waypoint(targetAllianceCFPose);
    List<Waypoint> waypointList = new ArrayList<Waypoint>();
    waypointList.add(waypoint);

    MultiDrivetrainRC command =
        new MultiDrivetrainRC(
            commandID,
            MultiDrivetrainRC.COMMAND_TYPE.ABSOLUTE,
            MultiDrivetrainRC.COMMAND_CONTENTS.TRANSLATION_AND_ROTATION,
            waypointList,
            rotationDirection,
            odometry,
            xyPIDSettings,
            rotationPIDSettings,
            1.0,
            -1);

    command.initializeCommand();
    robotDrivetrainCommandQueue.add(command);
  }

  /**
   * Add a drivetrain spin turn command to the robot command queue to turn the heading to the
   * closest multiple of 45-degrees.
   *
   * @param commandID ID of the command
   * @param rotationDirection Specify spin turn direction as Clockwise or Counterclockwise
   * @param maxCommandDuration_ms Maximum duration (ms) for this command
   */
  public void qDrivetrainSpinTurnToNearest45(
      String commandID,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      int maxCommandDuration_ms) {
    final double targetTolerance = 2.5; // Must be >0.0
    double deltaIMUHeadingDegrees;

    double currentIMUHeadingDegrees = getStoredIMUHeadingDegrees();

    if (rotationDirection == DrivetrainRC.ROTATION_DIRECTION.CLOCKWISE) {
      deltaIMUHeadingDegrees =
          (Math.ceil((currentIMUHeadingDegrees - targetTolerance) / 45.0) * 45.0)
              - 45.0
              - currentIMUHeadingDegrees;
    } else {
      deltaIMUHeadingDegrees =
          (Math.floor((getStoredIMUHeadingDegrees() + targetTolerance) / 45.0) * 45.0)
              + 45.0
              - currentIMUHeadingDegrees;
    }

    DrivetrainRC command =
        new DrivetrainRC(
            commandID,
            DrivetrainRC.COMMAND_TYPE.DELTA,
            DrivetrainRC.COMMAND_CONTENTS.ROTATION,
            new Pose2D(0.0, 0.0, Math.toRadians(deltaIMUHeadingDegrees)),
            rotationDirection,
            odometry,
            maxCommandDuration_ms);

    Log.i(TAG, "Queuing " + rotationDirection + " spin turn to nearest 45 degrees");

    command.initializeCommand();
    robotDrivetrainCommandQueue.add(command);
  }

  public void qDrivetrainSpinTurnToNearest45(
      String commandID,
      DrivetrainRC.ROTATION_DIRECTION rotationDirection,
      PIDSettings xyPIDSettings,
      PIDSettings rotationPIDSettings,
      int maxCommandDuration_ms) {
    final double targetTolerance = 2.5; // Must be >0.0
    double deltaIMUHeadingDegrees;

    double currentIMUHeadingDegrees = getStoredIMUHeadingDegrees();

    if (rotationDirection == DrivetrainRC.ROTATION_DIRECTION.CLOCKWISE) {
      deltaIMUHeadingDegrees =
          (Math.ceil((currentIMUHeadingDegrees - targetTolerance) / 45.0) * 45.0)
              - 45.0
              - currentIMUHeadingDegrees;
    } else {
      deltaIMUHeadingDegrees =
          (Math.floor((getStoredIMUHeadingDegrees() + targetTolerance) / 45.0) * 45.0)
              + 45.0
              - currentIMUHeadingDegrees;
    }

    DrivetrainRC command =
        new DrivetrainRC(
            commandID,
            DrivetrainRC.COMMAND_TYPE.DELTA,
            DrivetrainRC.COMMAND_CONTENTS.ROTATION,
            new Pose2D(0.0, 0.0, Math.toRadians(deltaIMUHeadingDegrees)),
            rotationDirection,
            odometry,
            xyPIDSettings,
            rotationPIDSettings,
            maxCommandDuration_ms);

    Log.i(TAG, "Queuing " + rotationDirection + " spin turn to nearest 45 degrees");

    command.initializeCommand();
    robotDrivetrainCommandQueue.add(command);
  }

  // endregion ** Robot Command Functions **

  /**
   * Call this once per loop to process robot commands. This function bulk reads the control and
   * expansion hubs first.
   */
  @Override
  public void processCommands() {
    boolean sendDriveTrainCommand = false;

    // Update the caches for both the control hub and expansion hub (if it was detected).
    bulkReadControlHub();
    bulkReadExpansionHub();

    // Update odometry if being used. This needs to be done before any robot commands are processed
    // to have a fresh pose available for use.
    // Also read IMU to get heading if 3-wheel odometry is not being used
    if (odometry != null) {
      if (odometry instanceof TwoWheelOdometry) {

        odometry.updateOdometry(readIMUHeading()); // Update cumulative translation movements
      } else {
        odometry.updateOdometry(Double.NaN); // Update cumulative translation movements
      }
    } else {
      readIMUHeading();
    }

    // Obtain any manual drivetrain movement request. These values would remain 0 in AUTO.
//    double joystickX = drivetrainManualMovement_X;
//    double joystickY = drivetrainManualMovement_Y;
//    double spin = drivetrainManualMovement_Spin;
    double joystickX;
    double joystickY;
    double spin;
    boolean commandRemoved = false;

    // If there is a command in the queue and there is a request to cancel the command,
    // then remove the first command. If there is a manual drivetrain movement, then clear
    // the queue.
//    if (!robotDrivetrainCommandQueue.isEmpty()) {
//      if (drivetrainManualMovementUpdated) {
//        robotDrivetrainCommandQueue.clear();
//        commandRemoved = true;
//      } else if (cancelCurrentDrivetrainRobotCommand) {
//        robotDrivetrainCommandQueue.remove(0);
//        cancelCurrentDrivetrainRobotCommand = false;
//
//        commandRemoved = true;
//      }
//    }

    // Look if there are still commands in the queue
    if (!robotDrivetrainCommandQueue.isEmpty()) {
      RobotCommand currentCommand = robotDrivetrainCommandQueue.get(0);

      // Run the processCommand() function of the current command.
      // A return value of true means it is complete, so remove it from the queue.
      // This has been checked for Generic and Delay commands.
      // This same flow should be utilized for custom commands created for each season
      //            boolean isCommandComplete = currentCommand.processCommand();

      // Check whether the current command has started yet.
      // If not, run the onStart() function.
      if (!currentCommand.hasCommandStarted()) {
        currentCommand.onStart();
      }

      currentCommand.processCommand(); // Run the current command

      // Check whether the current command is complete
      // If so, run the onComplete() function and then remove the command from the queue
      if (currentCommand.isComplete()) {
        currentCommand.onComplete();
        completedRCList.add(
            currentCommand.getCommandID()); // Save the command ID of this completed command
        robotDrivetrainCommandQueue.remove(0); // Remove the command from the queue
        currentCommand = null;
        commandRemoved = true;
        Log.i(TAG, "CoreRobot.java: Removing a completed command from robot command queue.");

        // Need to stop any drivetrain cmds (set to zeros)
      } else { // Existing command is not completed
        if (currentCommand instanceof DrivetrainRC) {
          Pose2D joystickValues = ((DrivetrainRC) currentCommand).getCurrentPIDControllerValues();

          switch (((DrivetrainRC) currentCommand).getCommandType()) {
            case TRANSLATION -> {
              joystickX = joystickValues.x;
              joystickY = joystickValues.y;
            }
            case ROTATION -> spin = joystickValues.heading;
            case TRANSLATION_AND_ROTATION -> {
              joystickX = joystickValues.x;
              joystickY = joystickValues.y;
              spin = joystickValues.heading;
            }
          }
          sendDriveTrainCommand = true;
        } else if (currentCommand instanceof MultiDrivetrainRC) {
          Pose2D joystickValues =
              ((MultiDrivetrainRC) currentCommand).getCurrentPIDControllerValues();

          switch (((MultiDrivetrainRC) currentCommand).getCommandType()) {
            case TRANSLATION -> {
              joystickX = joystickValues.x;
              joystickY = joystickValues.y;
            }
            case ROTATION -> spin = joystickValues.heading;
            case TRANSLATION_AND_ROTATION -> {
              joystickX = joystickValues.x;
              joystickY = joystickValues.y;
              spin = joystickValues.heading;
            }
          }
          sendDriveTrainCommand = true;
        }
        //                else {  // Commands that are not DrivetrainRC
        //                    processNonDrivetrainCommand(currentCommand);
        //                }
      }
    }

    if (commandRemoved) {
      joystickX = 0.0;
      joystickY = 0.0;
      spin = 0.0;
      sendDriveTrainCommand = true;
    }

    // If a manual drivetrain movement was detected
//    if (drivetrainManualMovementUpdated) {
//      sendDriveTrainCommand = true;
//      drivetrainManualMovementUpdated = false;
//    }

    // Send an updated drivetrain command if needed
//    if ((sendDriveTrainCommand) && (drivetrain != null)) {
    if (drivetrain != null) {
      // Calculate heading offset for alliance/field-centric CF if needed
//      double headingOffsetDegrees;
      if (fieldCentricDrive) {
        if ((odometry != null) && (odometry instanceof ThreeWheelOdometry)) {
          Pose2D currentPose2DEstimate = odometry.getPoseEstimate();
//          headingOffsetDegrees = Math.toDegrees(currentPose2DEstimate.heading);
          drivetrain.setCurrentIMUHeading(Math.toDegrees(currentPose2DEstimate.heading));
        }
        else {
//          headingOffsetDegrees = getStoredIMUHeadingDegrees();
          drivetrain.setCurrentIMUHeading(getStoredIMUHeadingDegrees());
        }
      }
//      else {
//        headingOffsetDegrees = 0.0;
//      }

      // Send the drive command
//      if (!isPTOEngaged) {
//        d.driveMecanumByCartesianENU(joystickX, joystickY, spin, headingOffsetDegrees);
//      }
    }

//    Msg.log("CoreRobot", "processCommands", "here");

    robotSystemList.values().forEach(CoreRobotSystem::processCommands);
    super.processCommands();
    drivetrain.processCommands();
  }
}
