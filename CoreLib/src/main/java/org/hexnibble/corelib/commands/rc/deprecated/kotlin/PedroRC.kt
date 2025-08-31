//package org.hexnibble.corelib.commands.rc.mechanisms
//
//import com.pedropathing.follower.Follower
//import com.pedropathing.pathgen.Path
//import com.pedropathing.pathgen.PathChain
//import org.hexnibble.corelib.commands.rc.LogPedroCoordinate
//import org.hexnibble.corelib.commands.rc.ParallelRaceRC
//import org.hexnibble.corelib.commands.rc.RC
//import org.hexnibble.corelib.commands.rc.SequentialRC
//import org.hexnibble.corelib.commands.rc.WaitForConditionRC
//import org.hexnibble.corelib.misc.Field
//import org.hexnibble.corelib.misc.Msg
//import java.util.function.BooleanSupplier
//
//class PedroRC @JvmOverloads constructor(
//  private val follower: Follower,
//  private val pathChain: PathChain,
//  private val holdEnd: Boolean = true,
//  commandID: String = "PedroRC"
//) : RC(commandID) {
//  @JvmOverloads
//  constructor(
//    follower: Follower,
//    path: Path,
//    holdEnd: Boolean = true,
//    commandID: String = "PedroRC"
//  ) : this(follower, PathChain(path), holdEnd, commandID)
//
//  enum class COORDINATE_LOG_OPTION {
//    ENTIRE_ROUTE,
//    AFTER_ODOMETRY_THRESHOLD,
//    NONE
//  }
//
//  override fun onStartCommand() = follower.followPath(pathChain, holdEnd)
//
//  override fun processCommand() {
//    if (!follower.isBusy) {
//      Msg.log(
//        ("Ending Pedro Pose="
//            + follower.pose.x
//            + ", "
//            + follower.pose.y
//            + ", "
//            + Math.toDegrees(follower.pose.heading))
//      )
//      val fieldCFPose = Field.convertPedroPoseToFieldPose(follower.pose)
//      Msg.log(
//        ("FieldCF Pose="
//            + fieldCFPose.x
//            + ", "
//            + fieldCFPose.y
//            + ", "
//            + Math.toDegrees(fieldCFPose.heading))
//      )
//      commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
//    }
//  }
//
//  companion object {
//    @JvmStatic
//    fun getBreakoutRC(
//      follower: Follower,
//      path: RC,
//      odometryThresholdForStartingSensorMonitoring: BooleanSupplier,
//      conditionForBreakout: BooleanSupplier,
//      logOption: COORDINATE_LOG_OPTION
//    ) : RC {
//      return when (logOption) {
//        COORDINATE_LOG_OPTION.NONE -> ParallelRaceRC(
//          path,
//          SequentialRC( // Monitor for odometry threshold
//            // Monitor for sensor condition
//            WaitForConditionRC(odometryThresholdForStartingSensorMonitoring),
//            WaitForConditionRC(conditionForBreakout)
//          )
//        )
//
//        COORDINATE_LOG_OPTION.AFTER_ODOMETRY_THRESHOLD -> ParallelRaceRC(
//          path,
//          SequentialRC( // Monitor for odometry threshold
//            WaitForConditionRC(odometryThresholdForStartingSensorMonitoring),
//            ParallelRaceRC(
//              LogPedroCoordinate(follower), // Monitor for sensor condition
//              WaitForConditionRC(conditionForBreakout)
//            )
//          )
//        )
//
//        COORDINATE_LOG_OPTION.ENTIRE_ROUTE -> ParallelRaceRC(
//          LogPedroCoordinate(follower),
//          path,
//          SequentialRC( // Monitor for odometry threshold
//            // Monitor for sensor condition
//            WaitForConditionRC(odometryThresholdForStartingSensorMonitoring),
//            WaitForConditionRC(conditionForBreakout)
//          )
//        )
//      }
//    }
//  }
//}