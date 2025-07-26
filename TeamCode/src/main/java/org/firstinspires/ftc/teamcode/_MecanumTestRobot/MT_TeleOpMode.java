package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadFWv3;
import org.hexnibble.corelib.wrappers.controller.ButtonToFunction;

@TeleOp(name = "MecanumTest TeleOp", group = "Linear OpMode")
public class MT_TeleOpMode extends CoreLinearOpMode {
    private MecanumTestRobot r;

    public MT_TeleOpMode() {
        super (OP_MODE_TYPE.TELE_OP, new ConfigFile(configFileName), new MTConstants());
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
        // Check motor powers
//        controller1.addActiveButtonGroup(
//              new ButtonToFunction(cross,
//                    () -> r.logMotorPowers()
//              )
//        );
    }

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
}
