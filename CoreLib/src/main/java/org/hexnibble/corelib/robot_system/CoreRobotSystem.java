//package org.hexnibble.corelib.robot_system;
//
//import androidx.annotation.NonNull;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import java.util.ArrayList;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map;
//import java.util.Objects;
//import java.util.Set;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.hexnibble.corelib.commands.rc.LogRC;
//import org.hexnibble.corelib.commands.rc.RC;
//import org.hexnibble.corelib.commands.rc.mechanisms.MoveLinearMechanismToPositionRC;
//import org.hexnibble.corelib.commands.rc.mechanisms.MoveServoRC;
//import org.hexnibble.corelib.misc.Msg;
//import org.hexnibble.corelib.wrappers.sensor.ColorSensorWrapper;
//import org.hexnibble.corelib.wrappers.sensor.CoreSensorWrapper;
//import org.hexnibble.corelib.wrappers.sensor.DistanceSensorWrapper;
//import org.hexnibble.corelib.wrappers.sensor.TouchSensorWrapper;
//import org.hexnibble.corelib.wrappers.servo.BaseServoWrapper;
//import org.hexnibble.corelib.wrappers.servo.CRServo;
//import org.hexnibble.corelib.wrappers.servo.RegularServo;
//
//public class CoreRobotSystem {
//  protected final HardwareMap hwMap;
//  private final String systemName;
//
//  protected final List<RC> systemRCList;
//
//  protected boolean manualMovementRequested;
//
//  //    protected final Map<String, BaseMotorWrapper> motorList;
//  protected final Map<String, BaseServoWrapper> servoList;
//  protected final Map<String, CoreSensorWrapper<?>> sensorList;
//  protected final Map<String, LinearMechanism> mechanismList;
//
//  public CoreRobotSystem(@NonNull HardwareMap hwMap, String systemName) {
//    Msg.log("CoreRobotSystem", "Constructor", "Starting");
//    this.hwMap = hwMap;
//    this.systemName = systemName;
//    manualMovementRequested = false;
//
//    // Instantiate lists here so we don't have to constantly check that the lists exist.
//    systemRCList = new ArrayList<>();
//    servoList = new HashMap<>();
//    sensorList = new HashMap<>();
//    mechanismList = new HashMap<>();
//  }
//
//  /** This should be called during robot setup to initialize any hardware that needs it. */
//  public void initializeSystem() {
//    servoList.values().forEach(BaseServoWrapper::initialize);
//  }
//
//  public void resetSystem() {
//    //        motorList.values().forEach(BaseMotorWrapper::reset);
//
//    servoList.values().forEach(BaseServoWrapper::reset);
//    sensorList.values().forEach(CoreSensorWrapper::reset);
//
//    mechanismList.values().forEach(LinearMechanism::reset);
//
//    systemRCList.clear();
//
//    manualMovementRequested = false;
//  }
//
//  public String getSystemName() {
//    return systemName;
//  }
//
//  /**
//   * Queues a RobotCommand in the system list. This is typically used during TeleOp. Existing
//   * commands are NOT cleared.
//   *
//   * @param rc
//   */
//  public void qSystemRC(RC rc) {
//    systemRCList.add(rc);
//  }
//
//  public void qSystemRC(RC rc, boolean clearExistingCommands) {
//    if (clearExistingCommands) {
//      systemRCList.clear();
//    }
//
//    systemRCList.add(rc);
//  }
//
//  // region ** Servo Functions **
//  protected RegularServo addRegularServo(
//      String servoName, BaseServoWrapper.SERVO_MODEL servoModel,
//      String servoEncoderName, DcMotorSimple.Direction encoderDirection,
//      double absoluteMinimumPosition, double absoluteMaximumPosition,
//      double minPosition, double angleAtMinimumPosition,
//      double maxPosition, double angleAtMaximumPosition)
//  {
//    RegularServo servo = new RegularServo(hwMap, servoName, servoModel,
//        servoEncoderName, encoderDirection,
//        absoluteMinimumPosition, absoluteMaximumPosition,
//        minPosition, angleAtMinimumPosition,
//        maxPosition, angleAtMaximumPosition);
//    servoList.put(servoName, servo);
//    return servo;
//  }
//
//  protected CRServo addCRServo(
//      String servoName, BaseServoWrapper.SERVO_MODEL servoModel,
//      double minSpeed, double maxSpeed)
//  {
//    CRServo servo = new CRServo(hwMap, servoName, servoModel);
//    servoList.put(servoName, servo);
//    return servo;
//  }
//
//  public String[] getServoNames() {
//    Set<String> listOfServoNames = servoList.keySet();
//    return listOfServoNames.toArray(new String[0]);
//  }
//
//  /**
//   * Read the servo's position from the attached encoder, if present.
//   *
//   * @param servoName Name of servo
//   * @return Servo position (0 - 1). Returns 0 if the servo does not exist or if there is no
//   *     encoder.
//   */
//  public double readServoPosition(String servoName) {
//    // Ensure this is a call to an existing regular servo (not a CR Servo)
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (!(servo instanceof RegularServo))
//      return 0.0;
//    else
//      return ((RegularServo) servo).readServoPositionFromEncoder();
//  }
//
//  public double readServoPositionDegrees(String servoName) {
//    // Ensure this is a call to an existing regular servo (not a CR Servo)
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (!(servo instanceof RegularServo))
//      return 0.0;
//    else
//      return ((RegularServo) servo).readServoPositionFromEncoderDegrees();
//  }
//
//  /**
//   * Get the target position (or speed if CR) of the servo.
//   *
//   * @param servoName Name of servo
//   * @return Target position (or speed if CR)
//   */
//  public double getServoTargetPosition(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null)
//      return servo.getTargetPosition();
//    else
//      return 0.0;
//  }
//
//  public double getServoTargetPositionDegrees(String servoName) {
//    RegularServo servo = (RegularServo) servoList.get(servoName);
//    if (servo != null)
//      return servo.getServoPositionDegrees();
//    else
//      return 0.0;
//  }
//
//  public void setServoPosition(String servoName, double position) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null)
//      servo.setServoPoint(position);
//  }
//
//  /**
//   * Set servo position in degrees as a RR action
//   *
//   * @param servoName Servo name
//   * @param targetPositionDegrees Target position (degrees)
//   * @param delayAfterMovement_ms Time to delay
//   * @return
//   */
//  public RC setServoPositionDegreesAsRC(
//      String servoName, double targetPositionDegrees, int delayAfterMovement_ms)
//  {
//    RegularServo servo = (RegularServo) servoList.get(servoName);
//    if (servo != null)
//      return new MoveServoRC(servoName, servo, targetPositionDegrees,
//            MoveServoRC.ServoUnit.DEGREES, delayAfterMovement_ms);
//    else
//      return new LogRC(
//          "CoreRobotSystem.setServoPositionDegreesAsRC unable to find servo " + servoName);
//  }
//
//  public RC setServoPositionAsRC(
//      String servoName, double targetPosition, int delayAfterMovement_ms)
//  {
//    RegularServo servo = (RegularServo) servoList.get(servoName);
//    if (servo != null)
//      return new MoveServoRC(servoName, servo, targetPosition,
//            MoveServoRC.ServoUnit.POSITION, delayAfterMovement_ms);
//    else
//      return new LogRC(
//          "CoreRobotSystem.setServoPositionDegreesAsRC unable to find servo " + servoName);
//  }
//
//  public void setServoPositionDegrees(String servoName, double targetPositionDegrees) {
//    RegularServo servo = (RegularServo) servoList.get(servoName);
//    if (servo != null) servo.setServoPositionDegrees(targetPositionDegrees);
//  }
//
//  /**
//   * Set the servo to the minimum position (or speed if CR).
//   *
//   * @param servoName Name of servo
//   */
//  public void setServoToMinPosition(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null)
//      servo.setServoToMinPosition();
//  }
//
//  /**
//   * Set the servo to the maximum position (or speed if CR).
//   *
//   * @param servoName Name of servo
//   */
//  public void setServoToMaxPosition(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.setServoToMaxPosition();
//  }
//
//  /**
//   * Increment the servo position (or speed if CR) up.
//   *
//   * @param servoName Name of servo
//   */
//  public void moveServoPositionUp(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.moveServoPositionUp();
//  }
//
//  public void moveServoPositionUp(String servoName, float positionIncrement) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.moveServoPositionUp(positionIncrement);
//  }
//
//  /**
//   * Decrement the servo position (or speed if CR) down.
//   *
//   * @param servoName Name of servo
//   */
//  public void moveServoPositionDown(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.moveServoPositionDown();
//  }
//
//  public void moveServoPositionDown(String servoName, float positionIncrement) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.moveServoPositionDown(positionIncrement);
//  }
//
//  public void disableServoPWM(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.disablePWM();
//  }
//
//  public void enableServoPWM(String servoName) {
//    BaseServoWrapper servo = servoList.get(servoName);
//    if (servo != null) servo.enablePWM();
//  }
//
//  // endregion ** Servo Functions **
//
//  // region ** Color Sensor Functions **
//  protected ColorSensorWrapper addColorSensor(String sensorName) {
//    ColorSensorWrapper sensor =
//        new org.hexnibble.corelib.wrappers.sensor.ColorSensorWrapper(hwMap, sensorName);
//    sensorList.put(sensorName, sensor);
//    return sensor;
//  }
//
//  /**
//   * This should be called to refresh the detected color. This can be called within an OpMode loop
//   * to update the detection.
//   */
//  public void refreshColorSensorDetection(String sensorName) {
//    ColorSensorWrapper sensor = (ColorSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) sensor.refreshColorSensorDetection();
//  }
//
//  public double readColorSensorDistance(String sensorName) {
//    // Check for null object because if there is a hardware malfunction, the sensor may not have
//    // been detected and added to the list.
//    // We do not want the robot to stop working because of this.
//    ColorSensorWrapper sensor = (ColorSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) return sensor.readDistanceFromSensor(DistanceUnit.MM);
//    else return -1.0;
//  }
//
//  public double getStoredColorSensorDistance_mm(String sensorName) {
//    // Check for null object because if there is a hardware malfunction, the sensor may not have
//    // been detected and added to the list.
//    // We do not want the robot to stop working because of this.
//    ColorSensorWrapper sensor = (ColorSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) return sensor.getStoredDistance_mm();
//    else return -1.0;
//  }
//
//  public boolean isColorDetected(String sensorName, ColorSensorWrapper.COLOR color) {
//    ColorSensorWrapper sensor = (ColorSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) {
//      boolean detectionResult = sensor.isColorDetected(color);
//      Msg.log(
//          getClass().getSimpleName(), "isColorDetected", color + " detected=" + detectionResult);
//      return detectionResult;
//    } else return false;
//  }
//
//  public float[] getColorSensorHSVData(String sensorName) {
//    ColorSensorWrapper sensor = (ColorSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) return sensor.getDetectedHSV();
//    else return new float[] {0.0f, 0.0f, 0.0f};
//  }
//
//  // endregion ** Color Sensor Functions **
//
//  // region ** Distance Sensor Functions **
//  public DistanceSensorWrapper addDistanceSensor(String sensorName) {
//    DistanceSensorWrapper sensor = new DistanceSensorWrapper(hwMap, sensorName);
//    sensorList.put(sensorName, sensor);
//    return sensor;
//  }
//
//  public double getDistanceSensorDistance(String sensorName) {
//    // Check for null object because if there is a hardware malfunction, the sensor may not have
//    // been detected and added to the list.
//    // We do not want the robot to stop working because of this.
//    DistanceSensorWrapper sensor = (DistanceSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) return sensor.getDistanceSensorReading();
//    else return -1.0;
//  }
//
//  // endregion ** Distance Sensor Functions **
//
//  // region ** Touch Sensor Functions **
//  public TouchSensorWrapper addTouchSensor(String sensorName) {
//    TouchSensorWrapper sensor = new TouchSensorWrapper(hwMap, sensorName);
//    sensorList.put(sensorName, sensor);
//    return sensor;
//  }
//
//  // Check for null object because if there is a hardware malfunction, the sensor may not have been
//  // detected and added to the list.
//  // We do not want the robot to stop working because of this.
//  public boolean isTouchSensorPressed(String sensorName) {
//    TouchSensorWrapper sensor = (TouchSensorWrapper) sensorList.get(sensorName);
//    if (sensor != null) return sensor.isPressed();
//    else return false;
//  }
//
//  // endregion ** Touch Sensor Functions **
//
//  // region ** Linear Mechanism **
//  protected void addLinearMechanism(String mechanismName, LinearMechanism mechanism) {
//    mechanismList.put(mechanismName, mechanism);
//  }
//
//  public LinearMechanism getLinearMechanism(String linearMechanismName) {
//    return mechanismList.get(linearMechanismName);
//  }
//
//  // Movement Functions
//  /**
//   * Move to a specified distance (based on lead distance/distance per rotation) RELATIVE TO encoder
//   * count of zero using specified power.
//   *
//   * @param position_mm Target position (mm)
//   * @param motorPower Motor power for this movement (0 - 1) This can be set false if the encoder
//   *     become out of sync (e.g. if a belt skips)
//   */
//  public double moveLinearMechanismToPosition_mm(
//      String linearMechanismName, double position_mm, double motorPower) {
//    return mechanismList
//        .get(linearMechanismName)
//        .moveToPosition_mm(position_mm, motorPower, true);
//  }
//
//  public RC moveLinearMechanismToPositionAsRC(String linearMechanismName, double position_mm) {
//    LinearMechanism mechanism = mechanismList.get(linearMechanismName);
//    if (mechanism != null)
//      return new MoveLinearMechanismToPositionRC(linearMechanismName, mechanism, position_mm, 5000);
//    else
//      return new LogRC(
//          "CoreRobotSystem.moveLinearMechanismToPositionAsRC unable to find mechanism "
//              + linearMechanismName);
//  }
//
//  /**
//   * Move toward the minimum distance (retraction).
//   *
//   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
//   */
//  public void moveLinearMechanismToMinPosition(String linearMechanismName, double motorPower) {
//    mechanismList.get(linearMechanismName).moveToMinPosition(motorPower);
//  }
//
//  public void moveLinearMechanismToMinPosition(
//      String linearMechanismName, double motorPower, boolean clearRCList) {
//    if (clearRCList) {
//      systemRCList.clear();
//    }
//    mechanismList.get(linearMechanismName).moveToMinPosition(motorPower);
//  }
//
//  /**
//   * Move to the max distance (extension).
//   *
//   * @param motorPower Motor power for this movement (0 - 1). Negative values will be made positive.
//   * @return New target position (mm)
//   */
//  public void moveLinearMechanismToMaxPosition(String linearMechanismName, double motorPower) {
//    mechanismList.get(linearMechanismName).moveToMaxPosition(motorPower);
//  }
//
//  public void moveLinearMechanismToMaxPosition(
//      String linearMechanismName, double motorPower, boolean clearRCList) {
//    if (clearRCList) {
//      systemRCList.clear();
//    }
//    mechanismList.get(linearMechanismName).moveToMaxPosition(motorPower);
//  }
//
//  /**
//   * Stop the mechanism at the current position. If power is set to zero, it may move based on
//   * gravity depending on its weight and gearing. If power is left as is, then the motor will remain
//   * running to try and maintain the position.
//   *
//   * @param setPowerToZero If true, will set the Power to 0.
//   */
//  public void stopLinearMechanism(String linearMechanismName, boolean setPowerToZero) {
//    mechanismList.get(linearMechanismName).stopMechanism(setPowerToZero);
//  }
//
//  /**
//   * Retrieve the current target position of the specified motor, in mm
//   *
//   * @return Current target position of the specified motor, in mm
//   */
//  public double getLinearMechanismCurrentTargetPosition_mm(String linearMechanismName) {
//    return mechanismList.get(linearMechanismName).getCurrentTargetPositionMotor1_mm();
//  }
//
//  /**
//   * Retrieve the current position of the specified motor (from the motor), in mm
//   *
//   * @return Current position of the specified motor, in mm
//   */
//  public double getLinearMechanismCurrentPosition_mm(String linearMechanismName) {
//    return mechanismList.get(linearMechanismName).getCurrentPositionMotor1_mm();
//  }
//
//  public double getLinearMechanismCurrentPosition_mm(String linearMechanismName, int motorNumber) {
//    if (motorNumber == 2)
//      return mechanismList.get(linearMechanismName).getCurrentPositionMotor2_mm();
//    else return mechanismList.get(linearMechanismName).getCurrentPositionMotor1_mm();
//  }
//
//  // Encoder Functions
//  public void setLinearMechanismEncoderZeroPositionCountOffset(String linearMechanismName) {
//    mechanismList.get(linearMechanismName).setEncoderZeroPositionCountOffset();
//  }
//
//  public void resetLinearMechanismEncoder(String linearMechanismName) {
//    mechanismList.get(linearMechanismName).resetEncoder();
//  }
//
//  // Motor Functions
//  public void setLinearMechanismMotorRunMode(String linearMechanismName, DcMotor.RunMode runMode) {
//    mechanismList.get(linearMechanismName).setSystemMotorRunMode(runMode);
//  }
//
//  public void setLinearMechanismMotorBrakeMode(
//      String linearMechanismName, DcMotor.ZeroPowerBehavior brakeMode) {
//    mechanismList.get(linearMechanismName).setSystemMotorBrakeMode(brakeMode);
//  }
//
//  public void setLinearMechanismMotorPower(String linearMechanismName, double motorPower) {
//    Objects.requireNonNull(mechanismList.get(linearMechanismName))
//        .setSystemMotorPower(motorPower);
//  }
//
//  public double getLinearMechanismMotorCurrent(String linearMechanismName) {
//    return mechanismList.get(linearMechanismName).getMotor1Current();
//  }
//
//  public double getLinearMechanismMotorCurrent(String linearMechanismName, int motorNumber) {
//    if (motorNumber == 2) return mechanismList.get(linearMechanismName).getMotor2Current();
//    else return mechanismList.get(linearMechanismName).getMotor1Current();
//  }
//
//  // endregion ** Linear Mechanism Functions **
//
//  /**
//   * Clear system robot command list.
//   */
//  public void clearSystemRCList() {
//    systemRCList.clear();
////    manualMovementRequested = true;
//  }
//
//  public boolean isSystemRCListEmpty() {
//    return systemRCList.isEmpty();
//  }
//
//  public void processCommands() {
//    if (!systemRCList.isEmpty()) {
//      //            RC rc = systemRCList.get(0);
//      //            boolean result = systemRCList.get(0).processCommand();
//      //            Msg.log("coreRobotSystem" + systemName, "processCommands", "commandID=" +
//      // rc.getCommandID() + ", result=" + result);
//      //            if (result) {
//      if (systemRCList.get(0).processRC()) {
//        //                Msg.log(getClass().getSimpleName() + systemName, "processCommands", "RC
//        // returned true so removing it from list");
//        systemRCList.remove(0);
//      }
//    }
//  }
//}
