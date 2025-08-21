package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;

/**
 * A path in robotics describes the route to get from a starting position to an end position.
 * For a drivetrain, this could be a line or various types of curves.
 */
public abstract class CorePath
{
    public enum ROTATION_DIRECTION {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    protected final Pose2D targetPose;      // Heading in radians
//    protected Pose2D holdPose;
    protected boolean isPathComplete;

    public CorePath(Pose2D targetPose) {
        this.targetPose = new Pose2D(targetPose);
//        this.holdPose = new Pose2D(targetPose);
        Msg.log(getClass().getSimpleName(), "Constructor", "Creating targetPose as " + targetPose);
    }

    public double getHeadingError(double currentIMUHeading) {
        return Field.addRadiansToIMUHeading(getTargetPose().heading, -currentIMUHeading);
    }


//    public boolean isPathComplete(Pose2D currentPose);

    public abstract boolean isPathComplete(Pose2D currentPose);

    public Pose2D getTargetPose() {
        return targetPose;
    }

//    public Pose2D getHoldPose() {
//        return holdPose;
//    }

    public abstract double getClosestInterpolatedTValue(Pose2D pose);
}
