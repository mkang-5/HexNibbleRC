package org.hexnibble.corelib.motion.path;

import java.util.ArrayList;
import java.util.Collections;

public class PathChain
{
    private final boolean holdPosition;
    private ArrayList<CorePath> pathChain = new ArrayList<>();

    public PathChain(boolean holdPosition, CorePath... paths) {
        this.holdPosition = holdPosition;
        Collections.addAll(pathChain, paths);
    }

    public PathChain(boolean holdPosition, ArrayList<CorePath> paths) {
        this.holdPosition = holdPosition;
        pathChain = paths;
    }

    public int size() {
        return pathChain.size();
    }

    /**
     * Get the specified CorePath from the chain.
     *
     * @param index 0-based index of the requested path. This will be clamped to prevent the
     *              index being out of bounds
     * @return returns the Path at the index.
     */
    public CorePath getPath(int index) {
        index = Math.clamp(index, 0, pathChain.size() - 1);
        return pathChain.get(index);
    }

    public boolean holdPosition() {
        return holdPosition;
    }
}
