package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import org.hexnibble.corelib.commands.rc.DrivetrainRC;
import org.hexnibble.corelib.commands.rc.InstantRC;
import org.hexnibble.corelib.commands.rc.RCController;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.motion.path.Line;
import org.hexnibble.corelib.motion.path.PathChain;
import org.hexnibble.corelib.opmodes.CoreAutoProgram;
import org.hexnibble.corelib.robot.CoreRobot;

public class MT_TestAutoProgram extends CoreAutoProgram {
   private MecanumTestRobot r;

   public MT_TestAutoProgram(CoreRobot robot, RCController rcController, Pose2D startingPose,
                             Runnable requestOpModeStop) {
      super(robot, rcController, startingPose, requestOpModeStop);

      r = (MecanumTestRobot) robot;
   }

   protected void createPaths() {
      Pose2D endPose = new Pose2D(0.0, 914.4, 0.0);

      DrivetrainRC moveToEnd = new DrivetrainRC(dtController, new PathChain(false,
            new Line(startingPose, endPose)));

//      InstantRC endProgram = new InstantRC(this::setProgramComplete);

      q(moveToEnd);
      q(new InstantRC(requestOpModeStop));
   }
}
