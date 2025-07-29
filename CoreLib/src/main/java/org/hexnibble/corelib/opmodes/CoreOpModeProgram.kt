//package org.hexnibble.corelib.opmodes
//
//import org.hexnibble.corelib.commands.rc.RCk
//import org.hexnibble.corelib.commands.rc.RCControllerk
//
//open class CoreOpModeProgram(
//  val rcController: RCControllerk
////  val follower: Follower,
////  val startingPose: Pose
//) {
//  var programComplete = false
//
//  init {
//    createPaths()
//  }
//
//  protected open fun createPaths() {}
//
//  protected fun q(vararg rc: RCk) = rcController.qRC(*rc)
//}