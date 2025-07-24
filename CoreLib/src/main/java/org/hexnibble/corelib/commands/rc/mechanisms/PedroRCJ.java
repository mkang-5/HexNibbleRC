//package org.hexnibble.corelib.commands.rc.mechanisms;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import java.util.function.BooleanSupplier;
//import org.hexnibble.corelib.commands.rc.LogPedroCoordinate;
//import org.hexnibble.corelib.commands.rc.ParallelRaceRC;
//import org.hexnibble.corelib.commands.rc.RC;
//import org.hexnibble.corelib.commands.rc.SequentialRC;
//import org.hexnibble.corelib.commands.rc.WaitForConditionRC;
//import org.hexnibble.corelib.misc.Field;
//import org.hexnibble.corelib.misc.Msg;
//import org.hexnibble.corelib.misc.Pose2D;
//
//@Deprecated(since = "5/25/25", forRemoval = true)
//public class PedroRCJ extends RC {
//  private final Follower pedroFollower;
//  private final PathChain pathChain;
//  private final boolean holdEnd;
//
//  public enum COORDINATE_LOG_OPTION {
//    ENTIRE_ROUTE,
//    AFTER_ODOMETRY_THRESHOLD,
//    NONE
//  }
//
//  public PedroRCJ(Follower follower, PathChain pathChain, boolean holdEnd) {
//    super("PedroRC", -1, null, null, true);
//    this.pedroFollower = follower;
//    this.pathChain = pathChain;
//    this.holdEnd = holdEnd;
//  }
//
//  public PedroRCJ(Follower follower, Path path, boolean holdEnd) {
//    this(follower, new PathChain(path), holdEnd);
//  }
//
//  public PedroRCJ(Follower follower, Path path) {
//    this(follower, path, true);
//  }
//
//  public PedroRCJ(Follower follower, PathChain pathChain) {
//    this(follower, pathChain, true);
//  }
//
//  @Override
//  protected void onStartCommand() {
//    pedroFollower.followPath(pathChain, holdEnd);
//  }
//
//  @Override
//  protected void processCommand() {
//    if (!pedroFollower.isBusy()) {
//      Msg.log("Ending Pedro Path. Pose="
//              + pedroFollower.getPose().getX() + ", "
//              + pedroFollower.getPose().getY() + ", "
//              + Math.toDegrees(pedroFollower.getPose().getHeading()));
//      Pose2D fieldCFPose = Field.convertPedroPoseToFieldPose(pedroFollower.getPose());
//      Msg.log("FieldCF Pose="
//              + fieldCFPose.x + ", "
//              + fieldCFPose.y + ", "
//              + Math.toDegrees(fieldCFPose.heading));
//      setCommandStatus(COMMAND_STATUSES.COMMAND_SUCCESSFULLY_COMPLETED);
//    }
//  }
//
////  public static RC getBreakoutRC(
////      Follower pedroFollower,
////      RC path,
////      BooleanSupplier odometryThresholdForStartingSensorMonitoring,
////      BooleanSupplier conditionForBreakout,
////      COORDINATE_LOG_OPTION logOption) {
////    return switch (logOption) {
////      case NONE ->
////          new ParallelRaceRC(
////              path,
////              new SequentialRC(
////                  // Monitor for odometry threshold
////                  new WaitForConditionRC(odometryThresholdForStartingSensorMonitoring),
////                  // Monitor for sensor condition
////                  new WaitForConditionRC(conditionForBreakout)));
////      case AFTER_ODOMETRY_THRESHOLD ->
////          new ParallelRaceRC(
////              path,
////              new SequentialRC(
////                  // Monitor for odometry threshold
////                  new WaitForConditionRC(odometryThresholdForStartingSensorMonitoring),
////                  new ParallelRaceRC(
////                      new LogPedroCoordinate(pedroFollower),
////                      // Monitor for sensor condition
////                      new WaitForConditionRC(conditionForBreakout))));
////      case ENTIRE_ROUTE ->
////          new ParallelRaceRC(
////              new LogPedroCoordinate(pedroFollower),
////              path,
////              new SequentialRC(
////                  // Monitor for odometry threshold
////                  new WaitForConditionRC(odometryThresholdForStartingSensorMonitoring),
////                  // Monitor for sensor condition
////                  new WaitForConditionRC(conditionForBreakout)));
////    };
////  }
//
//  //    @Override
//  //    public boolean processRC() {
//  //        if (!commandHasStarted) pedroFollower.followPath(path, holdEnd);
//  //        super.processRC();
//  //
//  //        if (!pedroFollower.isBusy()) {
//  //            Msg.log("Ending Pedro Line. Pose=" + pedroFollower.getPose().getX() + ", " +
//  // pedroFollower.getPose().getY() + ", " + Math.toDegrees(pedroFollower.getPose().getHeading()));
//  //            Pose2D fieldCFPose = Field.convertPedroPoseToFieldPose(pedroFollower.getPose());
//  //            Msg.log("FieldCF Pose=" + fieldCFPose.x + ", " + fieldCFPose.y + ", " +
//  // Math.toDegrees(fieldCFPose.heading));
//  //            return true;
//  //        } else return false;
//  //    }
//}
