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

//    @Override
//    public double getXError(Pose2D currentPose) {
//        return getTargetPose().x - currentPose.x;
//    }
//
//    @Override
//    public double getYError(Pose2D currentPose) {
//        return getTargetPose().y - currentPose.y;
//    }

    @Override
    public Pose2D getPoseError(Pose2D currentPose) {
        return new Pose2D(new Vector2D(
              getTargetPose().x - currentPose.x,
              getTargetPose().y - currentPose.y),
              0.0
        );
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

//    @Override
//    public double getClosestTValue(Pose2D pose) {
//        return 0.0;
//    }

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
