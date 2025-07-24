package org.hexnibble.corelib.opmodes

import org.hexnibble.corelib.commands.rc.RC
import org.hexnibble.corelib.commands.rc.RCController

open class CoreOpModeProgram(
  val rcController: RCController
//  val follower: Follower,
//  val startingPose: Pose
) {
  var programComplete = false

  init {
    createPaths()
  }

  protected open fun createPaths() {}

  protected fun q(vararg rc: RC) = rcController.qRC(*rc)
}