package org.hexnibble.corelib.robot;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public abstract class BaseOdometry {
  protected List<WheelMotor> odometryEncoderList;

  protected List<Double> previousEncoderPositionList;
  double previousIMUHeadingRadians;

  protected long lastEncoderPositionReadTime_ms = 0L;

  //    protected Pose2D deltaAllianceCentricPose = new Pose2D();
  protected Pose2D cumulativeAllianceCentricPose = new Pose2D();

  protected DecompositionSolver forwardSolver;

  BaseOdometry(List<WheelMotor> odometryWheelMotorList) {
    odometryEncoderList = odometryWheelMotorList;

    previousEncoderPositionList = List.of();
    previousIMUHeadingRadians = Double.NaN;
  }

  protected void clearOdometryWheelMotorList() {
    odometryEncoderList = List.of();
  }

  /**
   * Retrieve the encoder positions (in mm).
   *
   * @return List of encoder positions
   */
  protected ArrayList<Double> getEncoderPositionList_mm() {
    ArrayList<Double> odometryEncoderPositionList = new ArrayList<>();

    odometryEncoderList.forEach(
        wheelMotor -> odometryEncoderPositionList.add(wheelMotor.getCurrentPosition_mm()));
    lastEncoderPositionReadTime_ms = System.currentTimeMillis();

    return odometryEncoderPositionList;
  }

  /**
   * Retrieve the last time the encoder positions were read (system ms)
   *
   * @return Last encoder read time, in ms
   */
  public long getLastEncoderReadTime_ms() {
    return lastEncoderPositionReadTime_ms;
  }

  /** Reset odometry wheel encoders */
  public void resetEncoders() {
    odometryEncoderList.forEach(wheelMotor -> wheelMotor.resetEncoder());
  }

  public void resetHeading() {
    cumulativeAllianceCentricPose.heading = 0.0;
  }

  /**
   * Get current pose estimate in alliance CF.
   *
   * @return Pose estimate (alliance CF), with angle in radians.
   */
  public Pose2D getPoseEstimate() {
    return cumulativeAllianceCentricPose;
  }

  /**
   * Get the most recent delta pose estimate in alliance CF.
   *
   * @return Most recent delta pose estimate (alliance CF), with angle in radians.
   */
  //    public Pose2D getLastDeltaPoseEstimate() {
  //        return deltaAllianceCentricPose;
  //    }

  /**
   * Set current pose. This will reset the last delta pose and the last updated time to 0.
   *
   * @param newPose Alliance POV, with angle in radians
   */
  public void setPoseEstimate(Pose2D newPose) {
    previousEncoderPositionList = List.of();
    previousIMUHeadingRadians = Double.NaN;

    cumulativeAllianceCentricPose = newPose;
    //        deltaAllianceCentricPose = new Pose2D();
    lastEncoderPositionReadTime_ms = 0L;
  }

  /**
   * Update odometry with new encoder values. This function should be called each time through the
   * control loop to keep updating the odometry.
   *
   * @param IMUHeadingDegrees This is only used for 2-wheel odometry. It is ignored for 3-wheel.
   */
  public abstract void updateOdometry(double IMUHeadingDegrees);

  /**
   * Calculates the change in pose since the last update, in robot-centric POV. Derived from
   * Localization/ThreeTrackingWheelLocalizer.kt/calculatePoseDelta
   *
   * @param deltaPositionList List of position changes since the previous update (encoder readings,
   *     IMU heading if applicable).
   * @return Change in pose since the last update, in robot-centric POV.
   */
  protected Pose2D calculateDeltaPose(ArrayList<Double> deltaPositionList) {

    // Create an array of encoder changes since the last update
    double[][] deltaPositionArray = new double[deltaPositionList.size()][1];
    for (int i = 0; i < 3; i++) {
      deltaPositionArray[i][0] = deltaPositionList.get(i);
    }

    // Place the encoder changes into a matrix
    RealMatrix aMatrix = MatrixUtils.createRealMatrix(deltaPositionArray);
    aMatrix.transpose();
    RealMatrix solvedMatrix = forwardSolver.solve(aMatrix);

    double[][] calculatedDeltaPose = solvedMatrix.getData();

    return new Pose2D(
        calculatedDeltaPose[0][0], calculatedDeltaPose[1][0], calculatedDeltaPose[2][0]);
  }

  /**
   * Updates cumulative odometry to determine the current alliance-centric pose. Used by
   * updateOdometry function above. This assumes constant velocity during the measured time.
   */
  protected void updatePoseFromCumulativeOdometry(Pose2D deltaPose2D) {
    double dtheta = deltaPose2D.heading;

    double sinTerm;
    double cosTerm;

    if (Math.abs(dtheta) < 0.0000001) { // epsilonEquals
      sinTerm = 1.0 - (dtheta * dtheta / 6.0);
      cosTerm = dtheta / 2.0;
    } else {
      sinTerm = Math.sin(dtheta) / dtheta;
      cosTerm = (1.0 - Math.cos(dtheta)) / dtheta;
    }

    // RoadRunner Version
    Vector2D fieldPositionDelta =
        new Vector2D(
            sinTerm * deltaPose2D.x - cosTerm * deltaPose2D.y,
            cosTerm * deltaPose2D.x + sinTerm * deltaPose2D.y);

    Pose2D fieldPoseDelta =
        new Pose2D(
            fieldPositionDelta.rotateByAngleRadians(cumulativeAllianceCentricPose.heading),
            deltaPose2D.heading);

    cumulativeAllianceCentricPose.x += fieldPoseDelta.x;
    cumulativeAllianceCentricPose.y += fieldPoseDelta.y;

    cumulativeAllianceCentricPose.heading =
        Field.enforceIMUHeadingRangeRadians(
            cumulativeAllianceCentricPose.heading + fieldPoseDelta.heading);
  }
  /*    protected Pose2D updatePoseFromCumulativeOdometry(Pose2D fieldPose, Pose2D deltaPose2D) {
          double dtheta = deltaPose2D.heading;

          double sinTerm;
          double cosTerm;

          if (Math.abs(dtheta) < 0.0000001) {        // epsilonEquals
              sinTerm = 1.0 - (dtheta * dtheta / 6.0);
              cosTerm = dtheta / 2.0;
          }
          else {
              sinTerm = Math.sin(dtheta) / dtheta;
              cosTerm = (1.0 - Math.cos(dtheta)) / dtheta;
          }

          // RoadRunner Version
          Vector2D fieldPositionDelta = new Vector2D(
                  sinTerm * deltaPose2D.x - cosTerm * deltaPose2D.y,
                  cosTerm * deltaPose2D.x + sinTerm * deltaPose2D.y
          );

          Pose2D fieldPoseDelta = new Pose2D(fieldPositionDelta.rotateByAngleRadians(fieldPose.heading), deltaPose2D.heading);

          return new Pose2D(
                  fieldPose.x + fieldPoseDelta.x,
                  fieldPose.y + fieldPoseDelta.y,
                  Field.enforceIMUHeadingRangeRadians(fieldPose.heading + fieldPoseDelta.heading)
          );
      }
  */
}
