//package org.hexnibble.corelib.commands.rc
//
//import org.hexnibble.corelib.opmodes.CoreLinearOpMode
//import org.hexnibble.corelib.robot.CoreRobot
//import org.hexnibble.corelib.wrappers.controller.ControllerWrapper
//
//class RCControllerk(
//  val robot: CoreRobot,
////  val pedroFollower: Follower?,
//  private val opModeType: CoreLinearOpMode.OP_MODE_TYPE,
//  private val controller1: ControllerWrapper,
//  private val controller2: ControllerWrapper
//) {
//  private var activeRCList: MutableList<RCk> = ArrayList()
//
//  fun qRC(vararg rc: RCk) = activeRCList.addAll(rc)
//
//  fun processCommands() {
//    // Update the gamepad controllers and run any registered button commands
//    if (opModeType == CoreLinearOpMode.OP_MODE_TYPE.TELE_OP) {
//      controller1.updateGamepadData()
//      controller2.updateGamepadData()
//    }
//    // Process robot commands
//    // The robot also processes commands for each robot system
//    robot.processCommands()
//
//    // Update general running commands
//    if (activeRCList.isNotEmpty() && activeRCList[0].processRC()) {
//      activeRCList.removeAt(0)
//    }
////    pedroFollower?.update()
////    pedroFollower?.drawOnDashBoard()
//  }
//}