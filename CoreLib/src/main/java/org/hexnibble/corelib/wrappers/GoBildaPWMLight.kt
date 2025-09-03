//package org.hexnibble.corelib.wrappers
//
//import com.qualcomm.robotcore.hardware.HardwareMap
//import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
//import com.qualcomm.robotcore.hardware.ServoImplEx
//
//class GoBildaPWMLight(hwMap: HardwareMap, pwmLightName: String) {
//  private val light: ServoImplEx = hwMap.get(ServoImplEx::class.java, pwmLightName)
//
//  private fun setPWM() {
//    // The default PWM range of the Rev hubs are 600 - 2400 microseconds.
//    light.pwmRange = PwmRange(500.0, 2500.0)
//  }
//
//  fun initialize() = setPWM()
//
//  fun reinitialize() = setPWM()
//
//  fun turnLightOff() = light.setPosition(0.0)
//
//  fun turnLightOn() = light.setPosition(1.0)
//
//  fun setLightBrightness(brightness: Double) = light.setPosition(brightness)
//}