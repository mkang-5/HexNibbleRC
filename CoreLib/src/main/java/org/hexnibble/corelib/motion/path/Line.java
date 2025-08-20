package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;

public class Line extends CorePath
{
    Pose2D startPose;
//    Pose2D endPose;
    private final double targetHeadingToleranceTranslation_mm = 5.0;
    private final double targetHeadingToleranceRadians = Math.toRadians(2.0);
    final Vector2D line;

    public Line(Pose2D startPose, Pose2D targetPose) {
        super(targetPose);
        Msg.log(getClass().getSimpleName(), "Constructor",
              "Creating line path from " + startPose.x + ", " + startPose.y + ", "
                    + Math.toDegrees(startPose.heading) + " to "
                    + targetPose.x + ", " + targetPose.y + ", " + Math.toDegrees(targetPose.heading));
        this.startPose = new Pose2D(startPose);
        line = new Vector2D(targetPose.x - startPose.x, targetPose.y - startPose.y);
    }

    /**
     * Path completion occurs if:
     * (1) The distance and heading to the target are less than the threshold
     * @param currentPose The current pose with IMU Heading in degrees
     * @return Whether the path is complete
     */
    @Override
    protected void checkPathComplete(Pose2D currentPose) {
        isPathComplete = ((Math.abs(currentPose.getDistanceTo(targetPose)) < targetHeadingToleranceTranslation_mm)
            && (Math.abs(Field.addRadiansToIMUHeading(currentPose.heading, - targetPose.heading)) < targetHeadingToleranceRadians));
    }

//    @Override
//    public Pose2D getTargetPose() {
//        return endPose;
//    }

    /**
     * Calculate the parametric t on an interpolated version of this line segment that is closest
     * to the specified pose. If the point falls outside the segment, t will be clamped
     * between 0 and 1 to keep it on the segment.
     * @param pose
     * @return parametric t value that is closest to the specified pose (clamped to between 0 and 1)
     */
    public double getClosestInterpolatedTValue(Pose2D pose) {
        // Project pose onto line, as parameterized position d(t) = a + t * (b - a)
        // The formula is [(pose - startPose) . line] / (line . line)
        // Since a dot product of a vector with itself is the magnitude squared, use the latter
        double t = Vector2D.dotProduct(new Vector2D(pose.x - startPose.x, pose.y - startPose.y), line)
                / Math.pow(line.magnitude, 2);

        // Clamp t to stay on the line segment, in case the closest point was off the line segment
        return Math.clamp(t, 0.0, 1.0);
    }

    /**
     * Find the point on the line segment represented by the specified parametric t
     *
     * @param t Desired t value of the line segment.
     * @return Requested point
     */
    public Vector2D getPoint(double t) {
        return new Vector2D(startPose.x + (t * line.x),
                            startPose.y + (t * line.y));
    }
}
