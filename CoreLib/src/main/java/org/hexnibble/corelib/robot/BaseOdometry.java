package org.hexnibble.corelib.robot;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public abstract class BaseOdometry implements OdometryIface {
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
   * Retrieve the last time the encoder positions were read (system ms)
   *
   * @return Last encoder read time, in ms
   */
  public long getLastEncoderReadTime_ms() {
    return lastEncoderPositionReadTime_ms;
  }

  /** Reset odometry wheel encoders and pose
   */
  @Override
  public void resetEncodersAndPose() {
    odometryEncoderList.forEach(BaseMotorWrapper::resetEncoder);
    previousEncoderPositionList = List.of();
    previousIMUHeadingRadians = Double.NaN;
    setPoseEstimate(new Pose2D(0.0, 0.0, 0.0));
  }

  /**
   * Set current pose (alliance-centric CF).
   * This will reset the last delta pose and the last updated time to 0.
   *
   * @param newPose Alliance-centric X, Y coordinates (mm) and alliance-centric heading (radians)
   */
  @Override
  public void setPoseEstimate(Pose2D newPose) {
    previousEncoderPositionList = List.of();
    previousIMUHeadingRadians = Double.NaN;

    cumulativeAllianceCentricPose = newPose;
    //        deltaAllianceCentricPose = new Pose2D();
    lastEncoderPositionReadTime_ms = 0L;
  }

  /**
   * Get the current alliance-centric pose.
   *
   * @return Alliance CF pose. Heading is IMU-style in radians.
   */
  @Override
  public Pose2D getCurrentPose() {
    return cumulativeAllianceCentricPose;
  }

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

  /**
   * Get the current alliance-centric IMU heading.
   *
   * @return Alliance CF IMU heading (degrees).
   */
  @Override
  public double getIMUHeadingDegrees() {
    return Math.toDegrees(cumulativeAllianceCentricPose.heading);
  }

  /**
   * Obtain encoder counts as a list
   * @return List of encoder counts
   */
  @Override
  public ArrayList<Integer> getOdometryEncoderCounts() {
    ArrayList<Integer> odometryEncoderCounts = new ArrayList<>();

    odometryEncoderList.forEach(
          wheelMotor -> odometryEncoderCounts.add(wheelMotor.getCurrentPosition())
    );

    return odometryEncoderCounts;
  }


  /**
   * Retrieve the encoder positions (in mm).
   *
   * @return List of encoder positions
   */
  public ArrayList<Double> getOdometryEncoderPositions_mm() {
    ArrayList<Double> odometryEncoderPositionList = new ArrayList<>();

    odometryEncoderList.forEach(
          wheelMotor -> odometryEncoderPositionList.add(wheelMotor.getCurrentPosition_mm()));
    lastEncoderPositionReadTime_ms = System.currentTimeMillis();

    return odometryEncoderPositionList;
  }
}
