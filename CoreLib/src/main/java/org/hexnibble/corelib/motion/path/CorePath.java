package org.hexnibble.corelib.motion.path;

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

    protected final Pose2D targetPose;
    protected boolean isPathComplete;

    public CorePath(Pose2D targetPose) {
        this.targetPose = new Pose2D(targetPose);
    }

//    public boolean isPathComplete(Pose2D currentPose);

    public final boolean isPathComplete(Pose2D currentPose) {
        if (!isPathComplete) {
            checkPathComplete(currentPose);
        }

        return isPathComplete;
    }

    protected abstract void checkPathComplete(Pose2D currentPose);

    public Pose2D getTargetPose() {
        return targetPose;
    }

    public abstract double getClosestInterpolatedTValue(Pose2D pose);
}
