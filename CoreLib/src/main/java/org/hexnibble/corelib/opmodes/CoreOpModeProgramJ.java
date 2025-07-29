package org.hexnibble.corelib.opmodes;

import org.hexnibble.corelib.commands.rc.RC;
import org.hexnibble.corelib.commands.rc.RCController;


public class CoreOpModeProgramJ {
  private boolean programComplete;

  // Pedro
  protected RCController rcController;
//  protected Follower pedroFollower;
//  protected final Pose startingPedroPose;

  public CoreOpModeProgramJ(
        RCController rcController) { //, Follower pedroFollower, Pose startingPedroPose) {
    this.rcController = rcController;
//    this.pedroFollower = pedroFollower;
//    this.startingPedroPose = startingPedroPose;
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
