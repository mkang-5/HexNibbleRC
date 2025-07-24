package org.hexnibble.corelib.misc;


public final class Field {
  public static final double DEG_090_IN_RADS = Math.toRadians(90.0); // pi/2 Radians
  public static final double DEG_180_IN_RADS = Math.toRadians(180.0); // pi radians
  public static final double DEG_270_IN_RADS = Math.toRadians(270.0); // 3pi/2 radians
  public static final double DEG_360_IN_RADS = Math.toRadians(360.0); // 2pi radians
  public static final double TAU = 2.0 * Math.PI;

  public static final double FIELD_WIDTH_MM = 3585; // Measured our field 10/07/2024

  /**
   * Convert Cartesion coordinates using ENU (East-North_Up) convention to polar coordinates.
   * Remember Y joystick values need to be flipped when taken directly from the joystick.
   * Although absolute joystick X/Y values that do not lie on the X/Y axes will always be <1,
   * the conversion to polar coordinates (r) compensates for this such that r values ~1 (and thus
   * motor powers) can still be achieved.
   *
   * @param X Joystick movement in X direction, range -1.0 (left) to +1.0 (right)
   * @param Y Joystick movement in Y direction, range -1.0 (backward) to +1.0 (forward)
   */
  public static PolarCoords cartesianToPolarCoords(double X, double Y) {
    // Convert X and Y (robot-centric) to polar coordinates (robot-centric)
    // Remember 0 degrees lies on x-axis in polar coordinates
    PolarCoords polar = new PolarCoords();

    // First convert X, Y to polar r
    polar.r = Math.sqrt((Math.pow(X, 2.0) + Math.pow(Y, 2.0)));

    // Now calculate theta, but only if r is actually > 0. Otherwise there is no need.
    if (polar.r == 0) {
      return polar;
    }

    if (X == 0.0) // For values on y-axis (forward/backward)
    {
      if (Y < 0.0) { // Negative y values
        polar.theta = DEG_270_IN_RADS; // 270 degrees
      }
      else if (Y > 0.0) { // Positive y values
        polar.theta = DEG_090_IN_RADS; // 90 degrees
      }
    }
//    else if (Y == 0.0) // For values on x-axis (strafing left/right)
//    {
//      if (X < 0.0) { // Negative x values
//        polar.theta = DEG_180_IN_RADS; // 180 degrees
//      }
//    }
    else { // Diagonal movements
      polar.theta = Math.atan(Y / X);

      if (X < 0.0) {
        polar.theta += DEG_180_IN_RADS; // +180 degrees
      }
      else if (Y < 0.0) {
        polar.theta += DEG_360_IN_RADS; // +360 degrees
      }
    }
    return polar;
  }

  /**
   * Static function to add to (or subtract from) an IMU heading (+180 to -180 deg). Since positive
   * values are CCW rotations, adding will cause the new heading to be rotated CCW.
   *
   * @param IMUHeading IMU Heading (+180 to -180 deg). Positive values are CCW rotations (to left)
   *     as per right hand rule.
   * @param degrees Degrees to add (or subtract) using IMU heading range (addition will be a CCW
   *     rotation).
   * @return New heading in IMU reference system
   */
  public static double addDegreesToIMUHeading(double IMUHeading, double degrees) {
    double newHeading = IMUHeading + degrees;

    while ((newHeading > 180.0) || (newHeading < -180.0)) {
      if (newHeading > 180.0) {
        newHeading -= 360.0;
      }
      else { // newHeading < -180.0
        newHeading += 360.0;
      }
    }

    return newHeading;
  }

  /**
   * Static function to add to (or subtract from) an IMU heading (+180 to -180 deg). Since positive
   * values are CCW rotations, adding will cause the new heading to be rotated CCW.
   *
   * @param IMUHeadingRadians IMU Heading (+180 to -180 deg). Positive values are CCW rotations (to
   *     left) as per right hand rule.
   * @param radians Radians to add (or subtract) using IMU heading range (addition will be a CCW
   *     rotation).
   * @return New heading in IMU reference system
   */
  public static double addRadiansToIMUHeading(double IMUHeadingRadians, double radians) {
    double newHeading = IMUHeadingRadians + radians;

    while ((newHeading > Math.PI) || (newHeading < -Math.PI)) {
      if (newHeading > Math.PI) {
        newHeading -= TAU;
      }
      else { // newHeading < -Math.PI
        newHeading += TAU;
      }
    }

    return newHeading;
  }

  /**
   * Ensure given heading (in degrees) is within the range of 0 - 359.9 degrees
   *
   * @param headingDegrees Heading to range check
   * @return Heading within range of 0 - 359.9 degrees
   */
  public static double enforceHeadingRange360Degrees(double headingDegrees) {
    if (headingDegrees < 0.0) {
      return headingDegrees + 360.0;
    }
    else if (headingDegrees >= 360.0) {
      return headingDegrees - 360.0;
    }
    else {
      return headingDegrees;
    }
  }

  /**
   * Ensure given heading (in radians) is within the range of 0 - 2pi
   *
   * @param headingRadians Heading to range check
   * @return Heading within range of 0 - 2pi radians
   */
  public static double enforceHeadingRangeTauRadians(double headingRadians) {
    if (headingRadians < 0.0) {
      return headingRadians + TAU;
    }
    else if (headingRadians >= TAU) {
      return headingRadians - TAU;
    }
    else {
      return headingRadians;
    }
  }

  /**
   * Ensure given heading (in degrees) is within the range of +180 (CCW) to -180 (CW) degrees
   *
   * @param headingDegrees Heading to range check
   * @return Heading within range of +180 to -180 degrees
   */
  public static double enforceIMUHeadingRangeDegrees(double headingDegrees) {
    if (headingDegrees < -180.0) {
      return headingDegrees + 360.0;
    }
    else if (headingDegrees > 180.0) {
      return headingDegrees - 360.0;
    }
    else {
      return headingDegrees;
    }
  }

  /**
   * Ensure given heading (in radians) is within the range of +PI (CCW) to -PI (CW) radians
   *
   * @param headingRadians Heading to range check
   * @return Heading within range of +PI to -PI radians
   */
  public static double enforceIMUHeadingRangeRadians(double headingRadians) {
    if (headingRadians < -Math.PI) {
      return headingRadians + TAU;
    }
    else if (headingRadians > Math.PI) {
      return headingRadians - TAU;
    }
    else {
      return headingRadians;
    }
  }

  /**
   * Convert a field CF pose to alliance CF pose. Field CF has origin at bottom left corner
   * (audience blue corner). Alliance CF has origin in the middle of the field.
   */
  public static Pose2D convertFieldCFAbsoluteToAllianceCFAbsolute(
      Pose2D fieldCFPose, AllianceInfo.ALLIANCE_COLOR allianceColor) {
    double degrees90 = Math.toRadians(90);
    double rotationAngleRadians;

    if (allianceColor == AllianceInfo.ALLIANCE_COLOR.BLUE) {
      rotationAngleRadians = -degrees90;
    } else {
      rotationAngleRadians = degrees90;
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
   * Mirror red alliance poses (in field CF) to blue alliance poses. Headings must be IMU-style.
   *
   * @param redPose Red alliance pose to convert (in field CF)
   * @return Blue alliance pose (in field CF)
   */
  public static Pose2D mirrorRedToBlueAlliancePoses(Pose2D redPose) {
    redPose.x = (Field.FIELD_WIDTH_MM - redPose.x);
    redPose.heading = -redPose.heading;
    return redPose;
  }

  /**
   * Mirror red alliance poses (in field CF) to blue alliance poses. Headings must be in radians,
   * IMU-style.
   *
   * @param redPose Red alliance pose to convert (in field CF)
   * @return Blue alliance pose (in field CF)
   */
  public static Pose2D mirrorDiagonalRedToBlueAlliancePoses(Pose2D redPose) {
    Pose2D newPose = new Pose2D();
    newPose.x = (Field.FIELD_WIDTH_MM - redPose.x);
    newPose.y = (Field.FIELD_WIDTH_MM - redPose.y);
    newPose.heading = enforceIMUHeadingRangeRadians(redPose.heading + DEG_180_IN_RADS);

    Msg.log(
        "Converting Pose ("
            + redPose.x
            + ", "
            + redPose.y
            + ", "
            + redPose.heading
            + ") to ("
            + newPose.x
            + ", "
            + newPose.y
            + ", "
            + newPose.heading);
    return newPose;
  }

  //    /**
  //     * Convert field CF coordinates to RoadRunner CF coordinates.
  //     * Our field CF has the origin at the audience left corner, +x to the right, +y away from
  // the audience, and 0 radians on the +y.
  //     * RR uses a red alliance-centric CF with the origin in the middle of the field. Distances
  // are in inches.
  //     * RR headings are zero on field CF Y-axis (RR's X-axis) and increase counter-clockwise
  // (like polar).
  //     *
  //     * @param fieldX Starting pose X in field CF coordinates (mm).
  //     * @param fieldY Starting pose Y in field CF coordinates (mm).
  //     * @param fieldHdgRadians Headings should be field CF IMU-style, specified in radians.
  //     * @return RR Pose2d
  //     */
  //    static public Pose2d convertFieldPoseToRRPose(double fieldX, double fieldY, double
  // fieldHdgRadians) {
  //        final double rotationAngleRadians = Math.toRadians(90);
  //
  //        // Rotate the field CF 90 degrees CCW
  //        double RRCF_x = fieldX * Math.cos(rotationAngleRadians) + fieldY *
  // Math.sin(rotationAngleRadians);
  //        double RRCF_y = fieldX * -Math.sin(rotationAngleRadians) + fieldY *
  // Math.cos(rotationAngleRadians);
  //
  //        // Translate the origin (this can be thought of as the location of the fieldCF origin
  //        // in terms of the allianceCF)
  //        double translationAmount_mm = Field.FIELD_WIDTH_MM / 2.0;
  //        RRCF_x -= translationAmount_mm;
  //        RRCF_y += translationAmount_mm;
  //
  //        // Convert to inches
  //        RRCF_x /= 25.4;
  //        RRCF_y /= 25.4;
  //        double RRCF_heading = Field.enforceHeadingRangeTauRadians(fieldHdgRadians);
  //
  //        return new Pose2d(RRCF_x, RRCF_y, RRCF_heading);
  //    }

  //    /**
  //     * Convert field CF coordinates to RoadRunner CF coordinates.
  //     * Our field CF has the origin at the audience left corner, +x to the right, +y away from
  // the audience, and 0 radians on the +y.
  //     * RR uses a red alliance-centric CF with the origin in the middle of the field. Distances
  // are in inches.
  //     * RR headings zero on the positive X-axis and increasing counter-clockwise (like polar).
  //     *
  //     * @param fieldCFPose Starting pose in field CF coordinates (mm). Headings should be field
  // CF IMU-style, specified in radians.
  //     * @return RR Pose2d
  //     */
  //    static public Pose2d convertFieldPoseToRRPose(Pose2D fieldCFPose) {
  //        return convertFieldPoseToRRPose(fieldCFPose.x, fieldCFPose.y, fieldCFPose.heading);
  //    }

  //    public static Pose2D convertRRPoseToFieldPose(Pose2d RRPose) {
  //        // Convert to mm
  //        double FieldCF_x = RRPose.position.x * 25.4;
  //        double FieldCF_y = RRPose.position.y * 25.4;
  //
  //        // Translate the origin (this can be thought of as the location of the fieldCF origin
  //        // in terms of the allianceCF)
  //        double translationAmount_mm = Field.FIELD_WIDTH_MM / 2.0;
  //        FieldCF_x += translationAmount_mm;
  //        FieldCF_y -= translationAmount_mm;
  //
  //        // Rotate the field CF 90 degrees CW
  //        final double rotationAngleRadians = Math.toRadians(-90);
  //        FieldCF_x = FieldCF_x * Math.cos(rotationAngleRadians) + FieldCF_y *
  // Math.sin(rotationAngleRadians);
  //        FieldCF_y = FieldCF_x * -Math.sin(rotationAngleRadians) + FieldCF_y *
  // Math.cos(rotationAngleRadians);
  //
  //        return new Pose2D(FieldCF_x, FieldCF_y, RRPose.heading.toDouble());
  //    }

  /**
   * Convert Field CF to Pedro CF. Pedro has the same origin (audience-side left corner) as well as
   * X and Y axes. However, Pedro's heading has 0 radians on the X-axis, increasing CCW, and uses
   * radians. Pedro also uses inches instead of mm.
   *
   * @param fieldCFX_mm X coordinate, in mm
   * @param fieldCFY_mm Y coordinate, in mm
   * @param fieldCFHeadingRadians IMU-style with 0 radians being on Y axis going away from audience
   *     side.
   * @return Pedro Pose using inches for x and y, radians for heading
   */
//  public static Pose convertFieldPoseToPedroPose(
//      double fieldCFX_mm, double fieldCFY_mm, double fieldCFHeadingRadians) {
//    return new Pose(
//        fieldCFX_mm / 25.4,
//        fieldCFY_mm / 25.4,
//        Field.enforceHeadingRangeTauRadians(fieldCFHeadingRadians + Math.toRadians(90.0)));
//  }

  /**
   * Convert Field CF to Pedro CF. Pedro has the same origin (audience-side left corner) as well as
   * X and Y axes. However, Pedro's heading has 0-degrees on the X-axis, increasing CCW, and uses
   * radians. Pedro also uses inches instead of mm.
   *
   * @param fieldCFPose X and Y coordinates in mm, Heading is IMU-style with 0 radians being on Y
   *     axis going away from audience side.
   * @return Pedro Pose using inches for x and y, radians for heading
   */
//  public static Pose convertFieldPoseToPedroPose(Pose2D fieldCFPose) {
//    return new Pose(
//        fieldCFPose.x / 25.4,
//        fieldCFPose.y / 25.4,
//        Field.enforceHeadingRangeTauRadians(fieldCFPose.heading + Math.toRadians(90.0)));
//  }
//
//  public static Pose2D convertPedroPoseToFieldPose(Pose pedroPose) {
//    return new Pose2D(
//        pedroPose.getX() * 25.4,
//        pedroPose.getY() * 25.4,
//        Field.enforceHeadingRangeTauRadians(pedroPose.getHeading() - Math.toRadians(90.0)));
//  }
}
