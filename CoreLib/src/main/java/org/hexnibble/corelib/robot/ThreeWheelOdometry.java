package org.hexnibble.corelib.robot;

import android.util.Log;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public class ThreeWheelOdometry extends BaseOdometry {
  /**
   * This is the constructor for ThreeWheelOdometry.
   *
   * @param odometryWheelMountingLocationList List of odometry wheel mounting locations relative to
   *     the center of the robot. X is left/right. Y is forward/back.
   * @param odometryWheelMotorList List of motors into which odometry wheel encoders are attached.
   */
  public ThreeWheelOdometry(
      List<Pose2D> odometryWheelMountingLocationList, List<WheelMotor> odometryWheelMotorList) {
    super(odometryWheelMotorList);

    Log.i(Constants.TAG, "Starting ThreeWheelOdometry constructor.");

    Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

    for (int i = 0; i < 3; i++) {
      Vector2D orientationVector = odometryWheelMountingLocationList.get(i).getHeadingAsVector();
      Vector2D positionVector = odometryWheelMountingLocationList.get(i).getCoordsAsVector();
      inverseMatrix.setEntry(i, 0, orientationVector.x);
      inverseMatrix.setEntry(i, 1, orientationVector.y);
      inverseMatrix.setEntry(
          i, 2, positionVector.x * orientationVector.y - positionVector.y * orientationVector.x);
    }

    forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

    // Ensure the specified configuration can support full localization
    Log.i(Constants.TAG, "Checking forward solver is non-singular.");
    assert (forwardSolver.isNonSingular());
  }

  /**
   * Update odometry with new encoder values. This function should be called each time through the
   * control loop to keep updating the odometry.
   *
   * @param IMUHeadingDegrees This parameter is ignored for 3-wheel odometry
   */
  @Override
  public void updateOdometry(double IMUHeadingDegrees) {
    ArrayList<Double> currentEncoderPositionList = getOdometryEncoderPositions_mm();

    ArrayList<Double> deltaPositionList =
        new ArrayList<>(); // List to store the changes of each wheel since the last update

    if (!previousEncoderPositionList.isEmpty()) {
      for (int i = 0; i < 3; i++) { // Iterate for each of the 3 wheel encoders
        deltaPositionList.add(
            currentEncoderPositionList.get(i)
                - previousEncoderPositionList.get(
                    i)); // Calculate and store the change of each wheel
      }

      // Calculate the change in drivetrain pose since the last update
      Pose2D deltaDrivetrainPose = calculateDeltaPose(deltaPositionList);

      // Add this pose change to the running total
      updatePoseFromCumulativeOdometry(deltaDrivetrainPose);
      //            cumulativeAllianceCentricPose =
      // updatePoseFromCumulativeOdometry(cumulativeAllianceCentricPose, deltaDrivetrainPose);
    }

    previousEncoderPositionList = currentEncoderPositionList;
  }
}
