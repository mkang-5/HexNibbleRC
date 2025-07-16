package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.misc.Pose2D;

/**
 * A path in robotics describes the route to get from a starting position to an end position.
 * For a drivetrain, this could be a line or various types of curves.
 */
public interface CorePath
{
    public double getClosestInterpolatedTValue(Pose2D pose);
}
