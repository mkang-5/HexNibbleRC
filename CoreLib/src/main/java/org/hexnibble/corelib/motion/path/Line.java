package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;

public class Line extends CorePath
{
    Pose2D startPose;

    private final double targetHeadingToleranceTranslation_mm = 5.0;
    private final double targetHeadingToleranceRadians = Math.toRadians(2.0);
    final Vector2D line;
    private double lastTValue;

    public Line(Pose2D startPose, Pose2D targetPose) {
        super(targetPose);
        Msg.log(getClass().getSimpleName(), "Constructor",
              "Creating line path from " + startPose.x + ", " + startPose.y + ", "
                    + Math.toDegrees(startPose.heading) + " to "
                    + targetPose.x + ", " + targetPose.y + ", " + Math.toDegrees(targetPose.heading));
        this.startPose = new Pose2D(startPose);
        line = new Vector2D(targetPose.x - startPose.x, targetPose.y - startPose.y);
        lastTValue = 0.0;
    }

    @Override
    public Pose2D getPoseError(Pose2D currentPose) {
        Vector2D closestCurvePoint = new Vector2D(0.0, 0.0);
        double currentT = getClosestTValue(currentPose.getCoordsAsVector(), closestCurvePoint);
        return new Pose2D(new Vector2D(
              getTargetPose().x - currentPose.x,
              getTargetPose().y - currentPose.y),
              Field.addRadiansToIMUHeading(getHeading(currentT), -currentPose.heading)
        );
    }

    protected double getHeading(double t) {
        switch (headingInterpolation) {
            case FIXED -> {
                return startPose.heading;
            }
            case LINEAR -> {
                return Field.addRadiansToIMUHeading(startPose.heading, (t * (targetPose.heading - startPose.heading)));
            }
            default -> throw new IllegalStateException("Unexpected value: " + headingInterpolation);
        }
    }

    /**
     * Path completion occurs if:
     * (1) The distance and heading to the target are less than the threshold
     * @param currentPose The current pose with IMU Heading in degrees
     */
    @Override
    public boolean isPathComplete(Pose2D currentPose) {
        // Only check if a path is complete if it has not already been completed.
        if (!isPathComplete) {
            isPathComplete = ((Math.abs(currentPose.getDistanceTo(targetPose)) < targetHeadingToleranceTranslation_mm)
                  && (Math.abs(Field.addRadiansToIMUHeading(currentPose.heading, - targetPose.heading)) < targetHeadingToleranceRadians));
        }

        return isPathComplete;
    }

    /**
     * Find the point on the line segment represented by the specified parametric t
     *
     * @param t Desired t value of the line segment (0 - 1).
     * @return Requested point
     */
    public Vector2D getPoint(double t) {
        t = Math.clamp(t, 0, 1);
        return new Vector2D(startPose.x + (t * line.x),
                            startPose.y + (t * line.y));
    }

    /**
     * Calculate the parametric t on an interpolated version of this line segment that is closest
     * to the specified pose. If the point falls outside the segment, t will be clamped
     * between 0 and 1 to keep it on the segment.
     * @param currentPosition
     * @return parametric t value that is closest to the specified pose (clamped to between 0 and 1)
     */
//   @Override
    public double getClosestTValue(Vector2D currentPosition, Vector2D refClosestPointOnCurve) {
        // First determine a rough t-value that is closest to the current pose
        // This is done by starting with the previous t-value and checking values around it as well
        // Store the t-value that gives the minimum distance to the current pose
        final double STEP_SIZE = 0.02;
        final int MAX_ONE_TAIL_VALUES = 10;

        int numLowerValues = (int) (lastTValue / STEP_SIZE) - 1;
        int numUpperValues = (int) ((1.0 - lastTValue) / STEP_SIZE) - 1;

        numLowerValues = Math.clamp(numLowerValues, 0, MAX_ONE_TAIL_VALUES);
        numUpperValues = Math.clamp(numUpperValues, 0, MAX_ONE_TAIL_VALUES);

        double[] tValuesToTest = new double[numLowerValues + numUpperValues + 1];

        tValuesToTest[0] = lastTValue;

        for (int i = 1; i < (numLowerValues + 1); i++) {
            tValuesToTest[i] = lastTValue - (i * STEP_SIZE);
        }

        for (int i = 1; i < (numUpperValues + 1); i++) {
            tValuesToTest[i + numLowerValues] = lastTValue + (i * STEP_SIZE);
        }

        double closestDist = 1e9;
        double closestT = 0.0;

        Msg.log(getClass().getSimpleName(), "getClosestTValue", "numLowerValues=" + numLowerValues + ", numUpperValues=" + numUpperValues + ", tValuesToTest=" + tValuesToTest.length);

        for (double t : tValuesToTest) {
            Vector2D pointAtT = getPoint(t);
            double dist = Vector2D.getDistance(pointAtT, currentPosition);

            if (dist < closestDist) {
                closestDist = dist;
                closestT = t;
                refClosestPointOnCurve.setXY(pointAtT.x, pointAtT.y);
//            Msg.log(getClass().getSimpleName(), "getClosestTValue", "Setting closest point on curve to " + refClosestPointOnCurve);
            }
        }
        lastTValue = closestT;
        return closestT;
    }
}
