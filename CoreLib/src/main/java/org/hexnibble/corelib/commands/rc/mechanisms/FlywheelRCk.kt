//package org.hexnibble.corelib.commands.rc.mechanisms
//
//import org.hexnibble.corelib.commands.rc.kotlin.RCk
//import org.hexnibble.corelib.motion.pid.PIDController
//import org.hexnibble.corelib.motion.pid.PIDSettings
//import org.hexnibble.corelib.wrappers.motor.WheelMotor
//import java.util.function.Supplier
//
///**
// * RC to move a flywheel like for a shooter mechanism.
// * @param motor The motor to use.
// * @param rotationDirection The direction to spin the motor.
// * @param PIDSettings The PID constants to use for the flywheel
// * @param deltaTimeSupplier ??
// * @param targetRPM The target RPM to send the flywheel to
// * @param commandID The ID of the command.
// */
//class FlywheelRCk(
//  private val motor: WheelMotor,
//  private val rotationDirection: ROTATION_DIRECTION,
//  PIDSettings: PIDSettings,
//  private val deltaTimeSupplier: Supplier<Int>,
//  private val targetRPM: Int,
//  commandID: String = "FlywheelRC"
//) : RCk(commandID) {
//  enum class ROTATION_DIRECTION {
//    CLOCKWISE,
//    COUNTERCLOCKWISE
//  }
//
//  private var PIDController: PIDController
//  private var currentEncoderCount = 0
//  private var controlValue = 0.0
//
//  init {
//    // Create PID Controller
//    val targetToleranceRPM = 5.0
//    PIDController = PIDController(PIDSettings, targetToleranceRPM)
//  }
//
//  override fun processCommand() {
//    val deltaTimeBetweenReadings: Int = deltaTimeSupplier.get()
//
//    if (deltaTimeBetweenReadings != 0) {
//      val previousEncoderCount = currentEncoderCount
//      currentEncoderCount = motor.currentPosition
//
//      val countsPerMs =
//        (currentEncoderCount - previousEncoderCount).toDouble() / deltaTimeBetweenReadings
//      val currentRPM = countsPerMs / motor.effectiveCountsPerRev * 1000.0 * 60.0
//      controlValue = PIDController.calculateNewControlValue(targetRPM - currentRPM)
//      motor.setPower(controlValue)
//    }
//  }
//}