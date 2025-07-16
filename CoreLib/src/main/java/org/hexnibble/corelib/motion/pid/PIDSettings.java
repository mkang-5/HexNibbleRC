package org.hexnibble.corelib.motion.pid;

public class PIDSettings {
  public double Ks;
  public double Kp;
  public double Ki;
  public double Kd;

  public PIDSettings(double Ks, double Kp, double Ki, double Kd) {
    this.Ks = Ks;
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
  }
}
