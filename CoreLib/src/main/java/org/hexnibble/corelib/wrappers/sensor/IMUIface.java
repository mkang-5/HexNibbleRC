package org.hexnibble.corelib.wrappers.sensor;

public interface IMUIface {
    void resetIMUHeading();
    double refreshIMUHeading();
    double getStoredIMUHeadingDegrees();
}
