package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import static org.hexnibble.corelib.wrappers.controller.ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.commands.rc.mechanisms.FlywheelRC;
import org.hexnibble.corelib.motion.path.CorePath;
import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.wrappers.controller.ButtonToFunction;
import org.hexnibble.corelib.wrappers.controller.ControllerWrapper;

@TeleOp(name = "MecanumTest TeleOp", group = "Linear OpMode")
public class MT_TeleOpMode extends CoreLinearOpMode {
    private MecanumTestRobot r;

    public MT_TeleOpMode() {
        super (OP_MODE_TYPE.TELE_OP, new MTConfigFile(configFileName), new MTConstants());
    }

    @Override
    protected CoreRobot createRobotObject(HardwareMap hwMap) {
        return new MecanumTestRobot(hwMap);
    }

    @Override
    protected void initializeOpMode() {
        super.initializeOpMode();
        createControllersForTeleOp();

        r = (MecanumTestRobot) robot;
    }

    @Override
    protected void createControllersForTeleOp() {
        super.createControllersForTeleOp();

        // region ** Controller 1 **
        // qSpinTurn
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.left_bumper,
                    () -> r.drivetrain.qSpinTurnToNearest45(rcController,
                          r.getStoredIMUHeadingDegrees(),
                          CorePath.ROTATION_DIRECTION.COUNTERCLOCKWISE))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.right_bumper,
                    () -> r.drivetrain.qSpinTurnToNearest45(rcController,
                          r.getStoredIMUHeadingDegrees(),
                          CorePath.ROTATION_DIRECTION.CLOCKWISE))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.dpad_up,
                    () -> r.drivetrain.qTranslationUp(rcController, r.getRobotPoseEstimate()))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.dpad_right,
                    () -> r.drivetrain.qTranslationRight(rcController, r.getRobotPoseEstimate()))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.dpad_down,
                    () -> r.drivetrain.qTranslationDiagDown(rcController, r.getRobotPoseEstimate()))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.dpad_up, NEWLY_PRESSED, ControllerWrapper.OPTION_SHIFT,
                    () -> r.drivetrain.qTestCurveUp(rcController, r.getRobotPoseEstimate()))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.dpad_down, NEWLY_PRESSED, ControllerWrapper.OPTION_SHIFT,
                    () -> r.drivetrain.qTestCurveDown(rcController, r.getRobotPoseEstimate()))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.dpad_right, NEWLY_PRESSED, ControllerWrapper.OPTION_SHIFT,
                    () -> r.drivetrain.qTestCurveRight(rcController, r.getRobotPoseEstimate()))
        );
        controller1.addActiveButtonGroup(
              new ButtonToFunction(ControllerWrapper.BUTTON_NAME.cross,
                    () -> {
                        if (r.isFlyWheelActive()) {
                            r.turnOffFlyWheel();
                        }
                        else rcController.qRC(r.qFlyWheel(5000));
                    })
        );


        // Check motor powers
//        controller1.addActiveButtonGroup(
//              new ButtonToFunction(cross,
//                    () -> r.logMotorPowers()
//              )
//        );
    }
/*
    @Override
    protected void addTelemetryBody() {
        Pose2D pose = r.getCurrentPose();
        Pose2D poseVel = r.getCurrentPoseVelocity();
        telemetry.addData("Robot Pose: Alliance CF x (mm)=", "%.2f", pose.x);
        telemetry.addData("Robot Pose: Alliance CF y (mm)=", "%.2f", pose.y);
        telemetry.addData(
        "Robot Pose: Alliance CF hdg (deg)=", "%.2f", Math.toDegrees(pose.heading));
        telemetry.addData(
        "Robot Pose: Alliance CF hdg vel (deg/s)=", "%.2f", Math.toDegrees(poseVel.heading));

        OctoQuadFWv3.LocalizerStatus status = r.getOQStatus();
        telemetry.addLine("OQ Status=" + status);
        if (status != OctoQuadFWv3.LocalizerStatus.RUNNING) {
            android.util.Log.i("oq", "OQ Status=" + status);
        }
    }

 */
}
