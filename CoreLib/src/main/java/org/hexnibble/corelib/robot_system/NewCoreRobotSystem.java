package org.hexnibble.corelib.robot_system;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.wrappers.sensor.ColorSensorWrapper;
import org.hexnibble.corelib.wrappers.sensor.CoreSensorWrapper;
import org.hexnibble.corelib.wrappers.sensor.RevDistanceSensorWrapper;
import org.hexnibble.corelib.wrappers.sensor.LimelightWrapper;
import org.hexnibble.corelib.wrappers.sensor.TouchSensorWrapper;
import org.hexnibble.corelib.wrappers.servo.BaseServoWrapper;
import org.hexnibble.corelib.wrappers.servo.CRServo;
import org.hexnibble.corelib.wrappers.servo.RegularServo;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public abstract class NewCoreRobotSystem {
   protected final HardwareMap hwMap;
   private final String systemName;

   protected final Map<String, BaseServoWrapper> servoList;
   protected final Map<String, CoreSensorWrapper<?>> sensorList;

   public NewCoreRobotSystem(@NonNull HardwareMap hwMap, String systemName) {
      Msg.log("CoreRobotSystem", "Constructor", "Starting");
      this.hwMap = hwMap;
      this.systemName = systemName;

      // Instantiate lists here so we don't have to constantly check that the lists exist.
      servoList = new HashMap<>();
      sensorList = new HashMap<>();
   }

   /** This should be called during robot setup to initialize any hardware that needs it. */
   public void initializeSystem() {
      servoList.values().forEach(BaseServoWrapper::initialize);
   }

   /**
    * Reset the system. This is called at the beginning of an OpMode if the robot object is not
    * recreated.
    */
   public void resetSystem() {
      //        motorList.values().forEach(BaseMotorWrapper::reset);
      servoList.values().forEach(BaseServoWrapper::reset);
      sensorList.values().forEach(CoreSensorWrapper::reset);
   }

   public String getSystemName() {
      return systemName;
   }

   // region ** Servo Functions **
   protected RegularServo addRegularServo(
         String servoName, BaseServoWrapper.SERVO_MODEL servoModel,
         String servoEncoderName, DcMotorSimple.Direction encoderDirection,
         double absoluteMinimumPosition, double absoluteMaximumPosition,
         double minPosition, double angleAtMinimumPosition,
         double maxPosition, double angleAtMaximumPosition)
   {
      RegularServo servo = new RegularServo(hwMap, servoName, servoModel,
            servoEncoderName, encoderDirection,
            absoluteMinimumPosition, absoluteMaximumPosition,
            minPosition, angleAtMinimumPosition,
            maxPosition, angleAtMaximumPosition);
      servoList.put(servoName, servo);
      return servo;
   }

   public RegularServo getRegularServo(String servoName) {
      return (RegularServo) servoList.get(servoName);
   }

   protected CRServo addCRServo(
         String servoName, BaseServoWrapper.SERVO_MODEL servoModel,
         double minSpeed, double maxSpeed)
   {
      CRServo servo = new CRServo(hwMap, servoName, servoModel);
      servoList.put(servoName, servo);
      return servo;
   }

   public CRServo getCRServo(String servoName) {
      return (CRServo) servoList.get(servoName);
   }


   public String[] getServoNames() {
      Set<String> listOfServoNames = servoList.keySet();
      return listOfServoNames.toArray(new String[0]);
   }
   // endregion ** Servo Functions **

   // region ** Sensor Functions **
   public CoreSensorWrapper getSensor(String sensorName) {
      return sensorList.get(sensorName);
   }

   protected ColorSensorWrapper addColorSensor(String sensorName) {
      ColorSensorWrapper sensor =
            new org.hexnibble.corelib.wrappers.sensor.ColorSensorWrapper(hwMap, sensorName);
      sensorList.put(sensorName, sensor);
      return sensor;
   }

   public RevDistanceSensorWrapper addDistanceSensor(String sensorName) {
      RevDistanceSensorWrapper sensor = new RevDistanceSensorWrapper(hwMap, sensorName);
      sensorList.put(sensorName, sensor);
      return sensor;
   }

   public TouchSensorWrapper addTouchSensor(String sensorName) {
      TouchSensorWrapper sensor = new TouchSensorWrapper(hwMap, sensorName);
      sensorList.put(sensorName, sensor);
      return sensor;
   }

   public LimelightWrapper addLimelightSensor(String sensorName) {
      LimelightWrapper sensor = new LimelightWrapper(hwMap, sensorName);
      sensorList.put(sensorName, sensor);
      return sensor;
   }
   // endregion ** Sensor Functions **

   public void processCommands() {
   };
}
