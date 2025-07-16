package org.hexnibble.corelib.commands.rc

import com.pedropathing.follower.Follower
import org.hexnibble.corelib.misc.Field
import org.hexnibble.corelib.misc.Msg

/**
 * Logs the current coordinate of the robot.
 * @param pedroFollower The Pedro Follower object.
 * @param commandID The ID of the command.
 */
class LogPedroCoordinate @JvmOverloads constructor(
  private val pedroFollower: Follower,
  commandID: String = "LogPedroCoordinate"
) : RC(commandID) {
  override fun processCommand() {
    Msg.log(
      "Pedro Pose: " + pedroFollower.pose.x + ", " + pedroFollower.pose.y + ", "
          + Math.toDegrees(pedroFollower.pose.heading) + " deg. Pedro Velocity: "
          + pedroFollower.velocity.xComponent + ", " + pedroFollower.velocity.yComponent
          + ", deg/s" + Math.toDegrees(pedroFollower.velocity.theta)
    )
    val fieldCFPose = Field.convertPedroPoseToFieldPose(pedroFollower.pose)
    Msg.log(
      "FieldCF Pose: " + fieldCFPose.x + ", " + fieldCFPose.y + ", "
          + Math.toDegrees(fieldCFPose.heading)
    )
    commandStatus = COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED
  }
}