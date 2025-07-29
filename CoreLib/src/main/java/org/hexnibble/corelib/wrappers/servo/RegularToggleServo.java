//package org.hexnibble.corelib.wrappers.servo;
//
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class RegularToggleServo extends RegularServo {
//  protected final double minTogglePosition;
//  protected final double maxTogglePosition;
//
//  protected enum TOGGLE_POSITION {
//    MINIMUM,
//    MAXIMUM
//  }
//
//  protected TOGGLE_POSITION currentTogglePosition;
//
//  /**
//   * Full constructor providing parameters as positions (0 - 1; 0.5 is the middle). Min/max and
//   * toggle positions can also be specified for this constructor.
//   *
//   * @param hwMap Hardware map
//   * @param servoName Servo name
//   * @param servoModel Servo model
//   * @param encoderName Servo analog input (encoder) name
//   * @param encoderDirection Servo encoder direction
//   * @param minPosition Minimum position
//   * @param maxPosition Maximum position
//   * @param minTogglePosition Minimum position when toggling between two states
//   * @param maxTogglePosition Maximum position when toggling between two states
//   */
//  public RegularToggleServo(
//      HardwareMap hwMap,
//      String servoName,
//      SERVO_MODEL servoModel,
//      String encoderName,
//      DcMotorSimple.Direction encoderDirection,
//      double minPosition,
//      double maxPosition,
//      double minTogglePosition,
//      double maxTogglePosition) {
//    super(hwMap, servoName, servoModel, encoderName, encoderDirection, minPosition, maxPosition);
//
//    this.minTogglePosition = minTogglePosition;
//    this.maxTogglePosition = maxTogglePosition;
//  }
//
//  /**
//   * Constructor providing parameters as positions (0 - 1; 0.5 is the middle). Toggle positions can
//   * also be specified and for this constructor, will also be used as the minimum and maximum
//   * positions.
//   *
//   * @param hwMap Hardware map
//   * @param servoName Servo name
//   * @param servoModel Servo model
//   * @param encoderName Servo analog input (encoder) name
//   * @param encoderDirection Servo encoder direction
//   * @param minTogglePosition Minimum position when toggling between two states
//   * @param maxTogglePosition Maximum position when toggling between two states
//   */
//  public RegularToggleServo(
//      HardwareMap hwMap,
//      String servoName,
//      SERVO_MODEL servoModel,
//      String encoderName,
//      DcMotorSimple.Direction encoderDirection,
//      double minTogglePosition,
//      double maxTogglePosition) {
//    this(
//        hwMap,
//        servoName,
//        servoModel,
//        encoderName,
//        encoderDirection,
//        minTogglePosition,
//        maxTogglePosition,
//        minTogglePosition,
//        maxTogglePosition);
//  }
//
//  /** Toggle servo between min/max */
//  public void toggleServo() {
//    if (currentTogglePosition == TOGGLE_POSITION.MINIMUM) {
//      setServoPosition(maxTogglePosition);
//      currentTogglePosition = TOGGLE_POSITION.MAXIMUM;
//    } else {
//      setServoPosition(minTogglePosition);
//      currentTogglePosition = TOGGLE_POSITION.MINIMUM;
//    }
//  }
//
//  public void setServoToMinTogglePosition() {
//    setServoPosition(minTogglePosition);
//    currentTogglePosition = TOGGLE_POSITION.MINIMUM;
//  }
//
//  public void setServoToMaxTogglePosition() {
//    setServoPosition(maxTogglePosition);
//    currentTogglePosition = TOGGLE_POSITION.MAXIMUM;
//  }
//}
