package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.ConfigFile;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
import org.hexnibble.corelib.robot.CoreRobot;

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
    protected void addTelemetryBody() {
        Pose2D pose = r.getCurrentPose();
        telemetry.addData("Robot Pose: Alliance CF x (mm)=", "%.2f", pose.x);
        telemetry.addData("Robot Pose: Alliance CF y (mm)=", "%.2f", pose.y);
        telemetry.addData(
                "Robot Pose: Alliance CF hdg (deg)=", "%.2f", Math.toDegrees(pose.heading));

        addLoopTimeInfoToTelemetry(processLoopTime());
    }
}
