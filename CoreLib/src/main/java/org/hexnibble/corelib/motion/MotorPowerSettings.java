package org.hexnibble.corelib.motion;

public final class MotorPowerSettings
{
    public double LF, RF, LB, RB;

    public MotorPowerSettings() {
    }

    public MotorPowerSettings(MotorPowerSettings mPowers) {
        this.LF = mPowers.LF;
        this.RF = mPowers.RF;
        this.LB = mPowers.LB;
        this.RB = mPowers.RB;
    }

    public void reset() {
        this.LF = 0.0;
        this.RF = 0.0;
        this.LB = 0.0;
        this.RB = 0.0;
    }
}
