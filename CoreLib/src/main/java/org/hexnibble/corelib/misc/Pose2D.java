package org.hexnibble.corelib.misc;

/** A 2-D pose containing a 2-D location (x, y) and heading */
public class Pose2D {
  public double x; // Typically in mm
  public double y; // Typically in mm
  public double heading; // Typically in radians

  public Pose2D() {
    x = 0.0;
    y = 0.0;
    heading = 0.0;
  }

  public Pose2D(Pose2D pose) {
    x = pose.x;
    y = pose.y;
    heading = pose.heading;
  }

  public Pose2D(Vector2D vector_xy, double heading) {
    this.x = vector_xy.x;
    this.y = vector_xy.y;
    this.heading = heading;
  }

  public Pose2D(double x, double y, double headingRadians) {
    this.x = x;
    this.y = y;
    this.heading = headingRadians;
  }

  public String toString() {
    return "x=" + x + ", y=" + y + "hdg (deg)=" + Math.toDegrees(heading);
  }

  public Vector2D getCoordsAsVector() {
    return new Vector2D(x, y);
  }

  public Vector2D getHeadingAsVector() {
    return new Vector2D(Math.sin(heading), Math.cos(heading));
  }

  /**
   * Calculate the linear distance from this Pose2D coordinate to the target
   *
   * @param targetPose target location
   * @return linear distance
   */
  public double getDistanceTo(Pose2D targetPose) {
    double deltaX = targetPose.x - this.x;
    double deltaY = targetPose.y - this.y;
    return Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
  }

  /**
   * Convert field CF -> alliance CF pose. Field CF has origin at bottom left corner (audience blue
   * corner). Alliance CF has origin in the middle of the field. Headings must be in radians
   * (IMU-style)
   */
  public static Pose2D convertFieldCFAbsoluteToAllianceCFAbsolute(
      Pose2D fieldCFPose, AllianceInfo.ALLIANCE_COLOR allianceColor) {
    double degrees90inRadians = Math.toRadians(90);
    double rotationAngleRadians;

    if (allianceColor == AllianceInfo.ALLIANCE_COLOR.BLUE) {
      rotationAngleRadians = -degrees90inRadians;
    } else {
      rotationAngleRadians = degrees90inRadians;
    }

    // Rotate the field CF
    double allianceCF_x =
        fieldCFPose.x * Math.cos(rotationAngleRadians)
            + fieldCFPose.y * Math.sin(rotationAngleRadians);
    double allianceCF_y =
        fieldCFPose.x * -Math.sin(rotationAngleRadians)
            + fieldCFPose.y * Math.cos(rotationAngleRadians);
    double allianceCF_headingRadians =
        Field.enforceIMUHeadingRangeRadians(fieldCFPose.heading - rotationAngleRadians);

    // Translate the origin (this can be thought of as the location of the fieldCF origin
    // in terms of the allianceCF)
    double translationAmount_mm = Field.FIELD_WIDTH_MM / 2.0;

    if (allianceColor == AllianceInfo.ALLIANCE_COLOR.BLUE) {
      allianceCF_x += translationAmount_mm;
      allianceCF_y -= translationAmount_mm;
    } else {
      allianceCF_x -= translationAmount_mm;
      allianceCF_y += translationAmount_mm;
    }

    return new Pose2D(allianceCF_x, allianceCF_y, allianceCF_headingRadians);
  }

  /**
   * Convert alliance CF pose to field CF pose. Field CF has origin at bottom left corner (audience
   * blue corner). Alliance CF has origin in the middle of the field. Heading is IMU style in
   * radians.
   */
  public static Pose2D convertAllianceCFAbsoluteToFieldCFAbsolute(
      Pose2D allianceCFPose, AllianceInfo.ALLIANCE_COLOR allianceColor) {
    double degrees90 = Math.toRadians(90);
    double rotationAngleRadians;

    if (allianceColor == AllianceInfo.ALLIANCE_COLOR.BLUE) {
      rotationAngleRadians = degrees90;
    } else {
      rotationAngleRadians = -degrees90;
    }

    // Rotate the alliance CF
    double fieldCF_x =
        allianceCFPose.x * Math.cos(rotationAngleRadians)
            + allianceCFPose.y * Math.sin(rotationAngleRadians);
    double fieldCF_y =
        allianceCFPose.x * -Math.sin(rotationAngleRadians)
            + allianceCFPose.y * Math.cos(rotationAngleRadians);
    double fieldCF_headingRadians =
        Field.enforceIMUHeadingRangeRadians(allianceCFPose.heading - rotationAngleRadians);

    // Translate the origin (this can be thought of as the location of the allianceCF origin
    // in terms of the fieldCF)
    double translationAmount_mm = Field.FIELD_WIDTH_MM / 2.0;

    fieldCF_x += translationAmount_mm;
    fieldCF_y += translationAmount_mm;

    return new Pose2D(fieldCF_x, fieldCF_y, fieldCF_headingRadians);
  }

  /**
   * Convert a delta pose from Field CF to alliance CF. For this calculation, the heading would not
   * change.
   */
  public static Pose2D convertFieldCFDeltaToAllianceCFDelta(
      Pose2D fieldCFDeltaPose, AllianceInfo.ALLIANCE_COLOR allianceColor) {
    double degrees90 = Math.toRadians(90);
    double rotationAngleRadians;

    if (allianceColor == AllianceInfo.ALLIANCE_COLOR.BLUE) {
      rotationAngleRadians = -degrees90;
    } else {
      rotationAngleRadians = degrees90;
    }

    // Rotate the field CF
    double allianceCF_deltaX =
        fieldCFDeltaPose.x * Math.cos(rotationAngleRadians)
            + fieldCFDeltaPose.y * Math.sin(rotationAngleRadians);
    double allianceCF_deltaY =
        fieldCFDeltaPose.x * -Math.sin(rotationAngleRadians)
            + fieldCFDeltaPose.y * Math.cos(rotationAngleRadians);

    return new Pose2D(allianceCF_deltaX, allianceCF_deltaY, fieldCFDeltaPose.heading);
  }

  /**
   * Convert a *delta pose* from robot CF to an absolute location in alliance CF. The rotation angle
   * is taken from the current pose in alliance CF
   *
   * @return Returns the ending allianceCF pose
   */
  public static Pose2D convertRobotCFDeltaToAllianceCFAbsolute(
      Pose2D robotCFDeltaPose, Pose2D currentAllianceCFPose) {
    Pose2D newAllianceCFPose = new Pose2D();

    // First rotate the robot CF delta vector to alliance CF
    double allianceCFDelta_x =
        robotCFDeltaPose.x * Math.cos(-currentAllianceCFPose.heading)
            + robotCFDeltaPose.y * Math.sin(-currentAllianceCFPose.heading);
    double allianceCFDelta_y =
        robotCFDeltaPose.x * -Math.sin(-currentAllianceCFPose.heading)
            + robotCFDeltaPose.y * Math.cos(-currentAllianceCFPose.heading);

    // Then add to the current alliance pose
    newAllianceCFPose.x = currentAllianceCFPose.x + allianceCFDelta_x;
    newAllianceCFPose.y = currentAllianceCFPose.y + allianceCFDelta_y;
    newAllianceCFPose.heading =
        Field.addRadiansToIMUHeading(currentAllianceCFPose.heading, robotCFDeltaPose.heading);

    return newAllianceCFPose;
  }

  /**
   * Convert a *delta pose* from robot CF to a *delta pose* in alliance CF. The rotation angle is
   * taken from the current pose in alliance CF
   *
   * @return Returns the delta allianceCF pose
   */
  public static Pose2D convertRobotCFDeltaToAllianceCFDelta(
      Pose2D robotCFDeltaPose, Pose2D currentAllianceCFPose) {
    Pose2D newAllianceCFPose = new Pose2D();

    // Rotate the robot CF delta vector to alliance CF
    newAllianceCFPose.x =
        robotCFDeltaPose.x * Math.cos(-currentAllianceCFPose.heading)
            + robotCFDeltaPose.y * Math.sin(-currentAllianceCFPose.heading);
    newAllianceCFPose.y =
        robotCFDeltaPose.x * -Math.sin(-currentAllianceCFPose.heading)
            + robotCFDeltaPose.y * Math.cos(-currentAllianceCFPose.heading);
    newAllianceCFPose.heading =
        Field.addRadiansToIMUHeading(currentAllianceCFPose.heading, robotCFDeltaPose.heading);

    return newAllianceCFPose;
  }
}
