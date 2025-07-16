package org.hexnibble.corelib.misc;

import androidx.annotation.Nullable;

/** Holds the alliance info (color, side) Written by Ben K. 2022-23 */
public final class AllianceInfo {
  public enum ALLIANCE_COLOR {
    BLUE,
    RED
  }

  private static ALLIANCE_COLOR allianceColor;

  public enum ALLIANCE_SIDE {
    LEFT,
    RIGHT
  }

  private static ALLIANCE_SIDE allianceSide;

  public enum FIELD_HALF {
    NORTH,
    SOUTH
  }

  private static FIELD_HALF fieldHalf;

  private static boolean isInfoSet = false;

  public AllianceInfo() {}

  /**
   * Set AllianceInfo parameters
   *
   * @param color Blue or Red
   * @param side left or right
   */
  public static void setAllianceInfo(ALLIANCE_COLOR color, ALLIANCE_SIDE side) {
    allianceColor = color;
    allianceSide = side;

    isInfoSet = true;

    if (((allianceColor == ALLIANCE_COLOR.BLUE) && (allianceSide == ALLIANCE_SIDE.LEFT))
        || ((allianceColor == ALLIANCE_COLOR.RED) && (allianceSide == ALLIANCE_SIDE.RIGHT)))
      fieldHalf = FIELD_HALF.NORTH;
    else fieldHalf = FIELD_HALF.SOUTH;
  }

  public static void clearAllianceInfo() {
    allianceColor = null;
    allianceSide = null;
    fieldHalf = null;
    isInfoSet = false;
  }

  /**
   * Return Alliance Color
   *
   * @return Alliance Color
   */
  @Nullable
  public static ALLIANCE_COLOR getAllianceColor() {
    return allianceColor;
  }

  /**
   * Return Alliance Side
   *
   * @return Alliance Side
   */
  @Nullable
  public static ALLIANCE_SIDE getAllianceSide() {
    return allianceSide;
  }

  /**
   * Return field half (north or side)
   *
   * @return Field half
   */
  @Nullable
  public static FIELD_HALF getFieldHalf() {
    return fieldHalf;
  }

  /**
   * Queries whether the alliance color and side information has been set.
   *
   * @return True/false whether info has been set.
   */
  public static boolean isInfoSet() {
    return isInfoSet;
  }
}
