package org.hexnibble.corelib.wrappers

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap

class AnalogInputWrapper(sensorName: String, hwMap: HardwareMap) {
  private val analogSensor: AnalogInput = hwMap.get(AnalogInput::class.java, sensorName)
  fun getVoltage(): Double = analogSensor.voltage
}