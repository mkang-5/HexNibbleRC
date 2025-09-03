package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Msg;

public class AnalogSensorWrapper extends CoreSensorWrapper<AnalogInput>{
    public AnalogSensorWrapper(HardwareMap hwMap, String sensorName) {
        super(hwMap, AnalogInput.class, sensorName);

        Msg.log(
                getClass().getSimpleName(), "AnalogSensorWrapper", "Creating analog sensor " + sensorName);
    }

    public double getVoltage() {
        return sensor.getVoltage();
    }
}
