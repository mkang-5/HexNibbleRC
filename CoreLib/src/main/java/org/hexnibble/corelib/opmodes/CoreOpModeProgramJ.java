package org.hexnibble.corelib.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import org.hexnibble.corelib.commands.rc.RC;
import org.hexnibble.corelib.commands.rc.RCController;

@Deprecated(since = "5/25/25", forRemoval = true)
public class CoreOpModeProgramJ {
  private boolean programComplete;

  // Pedro
  protected RCController rcController;
  protected Follower pedroFollower;
  protected final Pose startingPedroPose;

  public CoreOpModeProgramJ(
      RCController rcController, Follower pedroFollower, Pose startingPedroPose) {
    this.rcController = rcController;
    this.pedroFollower = pedroFollower;
    this.startingPedroPose = startingPedroPose;
    this.programComplete = false;

    createPaths();
  }

  protected void createPaths() {}

  protected final void q(RC... rc) {
    rcController.qRC(rc);
  }

  public final boolean getProgramComplete() {
    return programComplete;
  }

  protected final void setProgramComplete() {
    programComplete = true;
  }
}
