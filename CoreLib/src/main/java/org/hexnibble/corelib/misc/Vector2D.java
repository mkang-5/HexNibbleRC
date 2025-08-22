package org.hexnibble.corelib.misc;

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

  public Vector2D rotateByAngleRadians(double angle_radians) {
    double newX = x * Math.cos(angle_radians) - y * Math.sin(angle_radians);
    double newY = x * Math.sin(angle_radians) + y * Math.cos(angle_radians);
    return new Vector2D(newX, newY);
  }
}
