package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.commands.rc.RCController;
import org.hexnibble.corelib.exception.StopOpModeException;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
import org.hexnibble.corelib.opmodes.CoreAutoProgram;
import org.hexnibble.corelib.robot.CoreRobot;

@Autonomous(name = "MecanumTest Auto", group = "Linear OpMode")
public class MT_AutoOpMode extends CoreLinearOpMode {
    protected CoreAutoProgram autoProgram;

    private MecanumTestRobot r;

    public MT_AutoOpMode() {
        super (OP_MODE_TYPE.AUTO, new MTConfigFile(configFileName), new MTConstants());
    }

    @Override
    protected CoreRobot createRobotObject(HardwareMap hwMap) {
        return new MecanumTestRobot(hwMap);
    }

//    @Override
//    public void runOpMode() {
//        initializeOpMode();
//
//        telemetry.addLine("Ready.");
//        telemetry.update();
//
//        waitForStart();
//
//        // Play pressed
//        if (opModeIsActive()) {
//            // Stuff to run before OpMode Loop
//            onPressPlay();
//
//            // Run until the end of AUTO or until STOP is pressed)
//            while (opModeIsActive()
//                  && !autoProgram.getProgramComplete()) {
//
//                if (processLoopTime()) {
//                    rcController.processCommands();
//
//                    if (!isStopRequested()) {
//                        createTelemetryMessageForEachLoop();
//                    }
//                }
//            }
//        }
//        onStopOpMode();
//    }

    @Override
    protected void initializeOpMode() {
        super.initializeOpMode();

        r = (MecanumTestRobot) robot;

        Msg.log(className, "initializeOpMode", "Creating Auto program");
//        if (AllianceInfo.getAllianceSide() == AllianceInfo.ALLIANCE_SIDE.LEFT) {
            autoProgram =
                  new MT_TestAutoProgram(r, rcController, startingFieldPose, this::requestOpModeStop);
//        }
//        else {
//            autoProgram =
//                  new ITD_RIGHT_NEW_OLD_AutoProgram(r, preloadChoice, rcController, pedroFollower, startingPedroPose);
//        }
    }

    @Override
    protected void promptForSupplementalAutoInitInfo() throws StopOpModeException {
        super.promptForSupplementalAutoInitInfo();
    }
}
