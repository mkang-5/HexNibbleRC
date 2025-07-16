package org.hexnibble.corelib.misc;

public class PolarCoords {
  public double r;
  public double theta; // radians

  public PolarCoords() {
    r = 0.0;
    theta = 0.0;
  }

  public PolarCoords(double r, double theta) {
    this.r = r;
    this.theta = theta;
  }
}
