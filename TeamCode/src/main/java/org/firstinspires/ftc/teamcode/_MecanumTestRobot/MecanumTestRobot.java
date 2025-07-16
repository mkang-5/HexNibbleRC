package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.robot.MecanumDrivetrain;
import org.hexnibble.corelib.robot.ThreeWheelOdometry;
import org.hexnibble.corelib.robot_system.CoreRobotSystem;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadFWv3;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadWrapper;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;

public class MecanumTestRobot extends CoreRobot {
    private OctoQuadWrapper oq;
    public MecanumTestRobot(HardwareMap hwMap) {
        super(hwMap, "MecanumTestRobot");
    }

    @Override
    public void initializeSystem() {
        oq = new OctoQuadWrapper(hwMap, "OctoQuad", 1, OctoQuadFWv3.EncoderDirection.FORWARD,
                2, OctoQuadFWv3.EncoderDirection.FORWARD,
                -141.9f, 0.0f);

        // Create drivetrain object
        drivetrain =
                new MecanumDrivetrain(hwMap,
                        "LFMotor", DcMotor.Direction.REVERSE,
                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,
                        "RFMotor", DcMotor.Direction.FORWARD,
                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
                        "LBMotor", DcMotor.Direction.REVERSE,
                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,
                        "RBMotor", DcMotor.Direction.FORWARD,
                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
                        BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_435, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                        1.0, TARGET_POSITION_TOLERANCE, 48.0);

        addRobotSystemToList("Drivetrain", drivetrain);
    }

    public Pose2D getCurrentPose() {
        return oq.getCurrentPose();
    }

    @Override
    public void processCommands() {
        boolean sendDriveTrainCommand = false;

        // Update the caches for both the control hub and expansion hub (if it was detected).
        bulkReadControlHub();
        bulkReadExpansionHub();

        // Update odometry if being used. This needs to be done before any robot commands are processed
        // to have a fresh pose available for use.
        // Also read IMU to get heading if 3-wheel odometry is not being used
        if (oq != null) {
            oq.readLocalizerData();

            // Send an updated drivetrain command if needed
            if (drivetrain != null) {
                // Calculate heading offset for alliance/field-centric CF if needed
                if (fieldCentricDrive) {
                    drivetrain.setCurrentIMUHeading(oq.getStoredIMUHeadingDegrees());
                }
            }
        }

        robotSystemList.values().forEach(CoreRobotSystem::processCommands);
        super.processCommands();
        drivetrain.processCommands();
    }
}
