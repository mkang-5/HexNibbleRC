package org.hexnibble.corelib.motion;

public class MotionProfile {
  protected double targetDistance_mm;
  protected double distanceDuringAcceleration_mm;
  protected double distanceDuringCruise_mm;
  protected double v_max;
  protected double a_max;
  protected double accelerationDuration_ms;
  protected double cruiseDuration_ms;
  protected double totalDuration_ms;
  protected double decelerationStartTime_ms;

  /**
   * Create a trapezoidal motion profile. All distances are in mm.
   *
   * @param targetDistance Distance to target (mm)
   * @param v_max Maximum velocity (mm/s). The theoretical v_max is wheel circumference * rpm / 60.
   *     The practical v_max should be decreased to about 80% of this value.
   * @param a_max Maximum acceleration (mm/s/s)
   */
  public MotionProfile(double targetDistance, double v_max, double a_max) {
    // Calculate how long it takes to get to v_max
    double acceleration_duration = v_max / a_max;

    // Check whether there is enough distance to reach v_max
    // s = (v_not * t) + 0.5at^2        v_not = 0
    double distance_at_v_max = 0.5 * a_max * acceleration_duration * acceleration_duration;
    double halfTargetDistance_mm = targetDistance / 2.0;

    // If the distance is not enough to reach v_max, accelerate only until the halfway point
    if (distance_at_v_max > halfTargetDistance_mm) {
      acceleration_duration = Math.sqrt(halfTargetDistance_mm / (0.5 * a_max));

      // Recalculate v_max
      v_max = a_max * acceleration_duration;

      // Calculate the modified distance traveled based on new time to accelerate
      distance_at_v_max = 0.5 * a_max * acceleration_duration * acceleration_duration;
    }

    // calculate the time that we're at max velocity
    double cruise_distance = targetDistance - (2.0 * distance_at_v_max);
    double cruise_duration = cruise_distance / v_max;

    this.targetDistance_mm = targetDistance;
    this.v_max = v_max;
    this.a_max = a_max;
    this.accelerationDuration_ms = acceleration_duration * 1000.0;
    this.cruiseDuration_ms = cruise_duration * 1000.0;
    this.totalDuration_ms =
        (acceleration_duration + cruise_duration + acceleration_duration) * 1000.0;
    this.decelerationStartTime_ms = (acceleration_duration + cruise_duration) * 1000.0;

    this.distanceDuringCruise_mm = v_max * cruise_duration;
    this.distanceDuringAcceleration_mm =
        0.5 * a_max * acceleration_duration * acceleration_duration;
  }

  /**
   * Returns the point (e.g. distance) along the motion profile corresponding to the elapsed time.
   *
   * @return
   */
  public double getProfiledValue(long elapsedTime_ms) {
    double elapsedTime_s = elapsedTime_ms / 1000.0;

    // Check where along the motion profile the current elapsed time is, and return the
    // corresponding
    // distance at that time point.
    if (elapsedTime_ms <= accelerationDuration_ms) { // Acceleration Phase
      //            Log.i(TAG, "Acceleration Phase: ElapsedTime=" + elapsedTime_ms + ", AccelDur=" +
      // accelerationDuration_ms);
      return 0.5 * a_max * elapsedTime_s * elapsedTime_s;
    } else if (elapsedTime_ms <= decelerationStartTime_ms) { // Cruise Phase
      double elapsedCruiseTime_s = (elapsedTime_ms - accelerationDuration_ms) / 1000.0;
      //            Log.i(TAG, "Cruise Phase: ElapsedTime=" + elapsedTime_ms + ",
      // elapsedCruiseTime=" + elapsedCruiseTime_s);

      return distanceDuringAcceleration_mm + v_max * elapsedCruiseTime_s;
    } else if (elapsedTime_ms <= totalDuration_ms) { // Deceleration Phase
      double elapsedDecelerationTime_s = (elapsedTime_ms - decelerationStartTime_ms) / 1000.0;
      //            Log.i(TAG, "Deceleration Phase: ElapsedTime=" + elapsedTime_ms + ",
      // elapsedDecelTime=" + elapsedDecelerationTime_s);

      return distanceDuringAcceleration_mm
          + distanceDuringCruise_mm
          + (v_max * elapsedDecelerationTime_s)
          - (0.5 * a_max * elapsedDecelerationTime_s * elapsedDecelerationTime_s);
    } else { // Elapsed time is greater than the calculated duration of this motion profile
      //            Log.i(TAG, "Past Phase");

      return targetDistance_mm;
    }
  }
}
