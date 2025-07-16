package org.hexnibble.corelib.robot;

import android.util.Log;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;

public class TwoWheelOdometry extends BaseOdometry {
  /**
   * This is the constructor for TwoWheelOdometry.
   *
   * @param odometryWheelMountingLocationList A list of odometry wheel mounting locations relative
   *     to the center of the robot. The first wheel is for X direction (left/right movements). The
   *     second wheel is for Y direction (forward/back movemetns).
   * @param odometryWheelMotorList A list of motor objects where the odometry wheel encoders are
   *     plugged in. The first motor port is for X direction. The second motor port is for Y
   *     direction.
   */
  public TwoWheelOdometry(
      List<Pose2D> odometryWheelMountingLocationList, List<WheelMotor> odometryWheelMotorList) {
    super(odometryWheelMotorList);

    Log.i(Constants.TAG, "Starting TwoWheelOdometry constructor.");

    Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

    for (int i = 0; i < 2; i++) {
      Vector2D orientationVector = odometryWheelMountingLocationList.get(i).getHeadingAsVector();
      Vector2D positionVector = odometryWheelMountingLocationList.get(i).getCoordsAsVector();
      inverseMatrix.setEntry(i, 0, orientationVector.x);
      inverseMatrix.setEntry(i, 1, orientationVector.y);
      inverseMatrix.setEntry(
          i, 2, positionVector.x * orientationVector.y - positionVector.y * orientationVector.x);
    }
    inverseMatrix.setEntry(2, 2, 1.0);

    forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

    // Ensure the specified configuration can support full localization
    Log.i(Constants.TAG, "Checking forward solver is non-singular.");
    assert (forwardSolver.isNonSingular());
  }

  /**
   * Update odometry with new encoder values. This function is specific for 2-wheel odometry and
   * should be called each time through the control loop to keep updating the odometry.
   *
   * @param IMUHeadingDegrees Heading (degrees) obtained from IMU
   */
  @Override
  public void updateOdometry(double IMUHeadingDegrees) {
    ArrayList<Double> currentEncoderPositionList = getEncoderPositionList_mm();
    ArrayList<Double> deltaPositionList =
        new ArrayList<>(); // List to store the changes of each wheel and heading since the last
    // update

    double IMUHeadingRadians = Math.toRadians(IMUHeadingDegrees);

    if (!previousEncoderPositionList.isEmpty()) {
      for (int i = 0; i < 2; i++) { // Iterate for each of the 2 wheel encoders
        deltaPositionList.add(
            currentEncoderPositionList.get(i)
                - previousEncoderPositionList.get(
                    i)); // Calculate and store the change of each wheel
      }

      // Add delta heading to the list
      double deltaHeadingRadians =
          Field.addRadiansToIMUHeading(IMUHeadingRadians, -previousIMUHeadingRadians);
      deltaPositionList.add(deltaHeadingRadians);

      // deltaPositionList.add(Field.addRadiansToIMUHeading(IMUHeadingRadians,
      // -previousIMUHeadingRadians));

      // Calculate the change in drivetrain pose since the last update
      Pose2D deltaDrivetrainPose = calculateDeltaPose(deltaPositionList);

      // Add this pose change to the running total
      updatePoseFromCumulativeOdometry(deltaDrivetrainPose);
      //            cumulativeAllianceCentricPose =
      // updatePoseFromCumulativeOdometry(cumulativeAllianceCentricPose, deltaDrivetrainPose);
    }

    previousEncoderPositionList = currentEncoderPositionList;
    previousIMUHeadingRadians = IMUHeadingRadians;
  }
}
