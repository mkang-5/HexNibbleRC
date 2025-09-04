package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.opmodes.CoreLinearOpMode;
import org.hexnibble.corelib.robot.CoreRobot;

@Autonomous(name = "MecanumTest Auto", group = "Linear OpMode")
public class MT_AutoOpMode extends CoreLinearOpMode {
    private MecanumTestRobot r;

    public MT_AutoOpMode() {
        super (OP_MODE_TYPE.AUTO, new MTConfigFile(configFileName), new MTConstants());
    }

    @Override
    protected CoreRobot createRobotObject(HardwareMap hwMap) {
        return new MecanumTestRobot(hwMap);
    }

    @Override
    protected void initializeOpMode() {
        super.initializeOpMode();

        r = (MecanumTestRobot) robot;
    }

    @Override
    protected void promptForSupplementalAutoInitInfo() {

    }
}
