package org.hexnibble.corelib.misc;

import static org.hexnibble.corelib.misc.Field.DEG_090_IN_RADS;
import static org.hexnibble.corelib.misc.Field.DEG_180_IN_RADS;
import static org.hexnibble.corelib.misc.Field.DEG_270_IN_RADS;
import static org.hexnibble.corelib.misc.Field.DEG_360_IN_RADS;

import androidx.annotation.NonNull;

/** A 2-D vector */
public class Vector2D {
  public double x;
  public double y;
  public double magnitude;

  public Vector2D(double x, double y) {
    this.x = x;
    this.y = y;
    magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public Vector2D(Vector2D vector) {
    x = vector.x;
    y = vector.y;
    magnitude = vector.magnitude;
  }

  @NonNull
  @Override
  public String toString() {
    return "(" + x + ", " + y + ")";
  }

  public void setX(double x) {
    this.x = x;
    magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public void setY(double y) {
    this.y = y;
    magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public void setXY(double x, double y) {
    this.x = x;
    this.y = y;
    magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }

  public static double dotProduct(Vector2D vector1, Vector2D vector2) {
    return (vector1.x * vector2.x) + (vector1.y * vector2.y);
  }

  public static double getDistance(Vector2D vector1, Vector2D vector2) {
    return Math.sqrt(
        ((vector2.x - vector1.x) * (vector2.x - vector1.x))
            + ((vector2.y - vector1.y) * (vector2.y - vector1.y)));
  }

  public double getPolarHeadingRadians() {
    double theta = 0.0;

    if (x == 0.0) // For values on y-axis (forward/backward)
    {
      if (y < 0.0) { // Negative y values
        theta = DEG_270_IN_RADS; // 270 degrees
      }
      else if (y > 0.0) { // Positive y values
        theta = DEG_090_IN_RADS; // 90 degrees
      }
    }
    else {    // Other movements
      theta = Math.atan(y / x);

      if (x < 0.0) {
        theta += DEG_180_IN_RADS; // +180 degrees
      }
      else if (y < 0.0) {
        theta += DEG_360_IN_RADS; // +360 degrees
      }
    }
    return theta;
  }

  // Return the IMU heading of this vector
  public double getIMUHeading() {
    double theta = 0.0;
    if (x == 0.0) // For values on y-axis (forward/backward)
    {
      if (y < 0.0) { // Negative y values
        theta = DEG_180_IN_RADS;
      }
    }
    else {    // Other movements
      theta = Math.atan(y / x);

      if (x < 0.0) {
        theta += DEG_090_IN_RADS;
      }
      else if (y < 0.0) {
        theta += DEG_270_IN_RADS;
      }
    }
    return theta;
  }

  public Vector2D rotateByAngleRadians(double angle_radians) {
    double newX = x * Math.cos(angle_radians) - y * Math.sin(angle_radians);
    double newY = x * Math.sin(angle_radians) + y * Math.cos(angle_radians);
    return new Vector2D(newX, newY);
  }
}
