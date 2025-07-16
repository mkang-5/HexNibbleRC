package org.hexnibble.corelib.motion;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;

@Deprecated(since = "5/30/25")
public class Waypoint {
  protected Pose2D allianceCFSegmentStartingPose;
  protected Pose2D allianceCFSegmentTargetPose;

  /**
   * Waypoint constructor takes a Pose2D with alliance CF coordinates and IMU-style heading
   * (radians)
   */
  public Waypoint(Pose2D targetPose) {
    allianceCFSegmentTargetPose = targetPose;
  }

  /**
   * Waypoint constructor takes alliance CF coordinates and IMU-style heading (degrees). The heading
   * will be converted to radians before being stored.
   *
   * @param targetX X, alliance CF
   * @param targetY Y, alliance CF
   * @param targetHeadingDegrees IMU-style heading (degrees)
   */
  public Waypoint(double targetX, double targetY, double targetHeadingDegrees) {
    allianceCFSegmentTargetPose =
        new Pose2D(targetX, targetY, Math.toRadians(targetHeadingDegrees));
  }

  public Pose2D getStartingPose() {
    return allianceCFSegmentStartingPose;
  }

  public void setStartingPose(Pose2D startingPose) {
    allianceCFSegmentStartingPose = new Pose2D(startingPose);
  }

  /**
   * Get the waypoint/target pose (alliance CF).
   *
   * @return Alliance CF target pose
   */
  public Pose2D getTargetPose() {
    return allianceCFSegmentTargetPose;
  }

  /**
   * Set target pose (alliance CF).
   *
   * @param targetPose Target pose (alliance CF)
   */
  public void setTargetPose(Pose2D targetPose) {
    allianceCFSegmentTargetPose = targetPose;
  }

  public double getDeltaX() {
    return allianceCFSegmentTargetPose.x - allianceCFSegmentStartingPose.x;
  }

  public double getDeltaY() {
    return allianceCFSegmentTargetPose.y - allianceCFSegmentStartingPose.y;
  }

  public double getDeltaHeadingRadians() {
    return Field.enforceIMUHeadingRangeRadians(
        allianceCFSegmentTargetPose.heading - allianceCFSegmentStartingPose.heading);
  }

  public double getSegmentDistance_mm() {
    double deltaX = allianceCFSegmentTargetPose.x - allianceCFSegmentStartingPose.x;
    double deltaY = allianceCFSegmentTargetPose.y - allianceCFSegmentStartingPose.y;
    return Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
  }
}
