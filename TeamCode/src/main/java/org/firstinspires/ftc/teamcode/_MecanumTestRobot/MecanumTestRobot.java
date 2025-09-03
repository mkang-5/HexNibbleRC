package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.commands.rc.mechanisms.FlywheelRC;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.robot.TwoWheelOdometry;
import org.hexnibble.corelib.robot.drivetrain.MecanumDrivetrain;
import org.hexnibble.corelib.robot_system.FlyWheelSystem;
import org.hexnibble.corelib.robot_system.NewCoreRobotSystem;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadFWv3;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadWrapper;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.motor.WheelMotor;
import org.hexnibble.corelib.wrappers.sensor.IMUIface;
import org.hexnibble.corelib.wrappers.sensor.IMUWrapper;

import java.util.Arrays;

public class MecanumTestRobot extends CoreRobot {
    private FlyWheelSystem flyWheelSystem;

    public MecanumTestRobot(HardwareMap hwMap) {
        super(hwMap, "MecanumTestRobot");
    }

    @Override
    public void initializeSystem() {
        boolean useOQ = true;

        if (useOQ) {
            // Create drivetrain object
            drivetrain = new MecanumDrivetrain(hwMap,
                  "LFMotor", DcMotor.Direction.REVERSE,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotorSimple.Direction.REVERSE,

                  "RFMotor", DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
                  "LBMotor", DcMotor.Direction.REVERSE,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,

                  "RBMotor", DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotorSimple.Direction.FORWARD,

                  BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_435, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                  1.0, TARGET_POSITION_TOLERANCE, 48.0);

            Msg.logIfDebug(className, "initializeSystem", "Creating OctoQuad odometry object.");
            odometry = new OctoQuadWrapper(hwMap, "OctoQuad", 0, OctoQuadFWv3.EncoderDirection.FORWARD,
                1, OctoQuadFWv3.EncoderDirection.FORWARD,
                -141.9f, 0.0f);

            IMU = (IMUIface) odometry;
        }
        else {
            // Create drivetrain object
            drivetrain = new MecanumDrivetrain(hwMap,
                  "LFMotor", DcMotor.Direction.REVERSE,
                  BaseMotorWrapper.ENCODER.GO_BILDA_ODOPOD, DcMotorSimple.Direction.REVERSE,

                  "RFMotor", DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
                  "LBMotor", DcMotor.Direction.REVERSE,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,

                  "RBMotor", DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.ENCODER.GO_BILDA_ODOPOD, DcMotorSimple.Direction.REVERSE,

                  BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_435, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                  1.0, TARGET_POSITION_TOLERANCE, 48.0);

            // Create odometry object
            Msg.logIfDebug(className, "initializeSystem", "Creating odometry object.");
            odometry = new TwoWheelOdometry(
                Arrays.asList(
                      new Pose2D(
                            -141.9,
                            0.0,
                            Math.PI / 2.0), // Location of LR odometry wheel - x movements
                      new Pose2D(
                            141.9,
                            0.0,
                            0.0) // Location of FB odometry wheel - y movements
                ),
                Arrays.asList(
                      drivetrain.getWheelMotorObject(
                            MecanumDrivetrain.WHEEL_MODULE_NAME.LF), // LR odometry wheel (x movements)
                      drivetrain.getWheelMotorObject(
                            MecanumDrivetrain.WHEEL_MODULE_NAME.RB) // FB odometry wheel (y movements)
                ));

            IMU = new IMUWrapper(hwMap, IMU_NAME, CH_LOGO_FACING_DIRECTION, CH_USB_FACING_DIRECTION);
        }

        WheelMotor wheelMotor1 = new WheelMotor(hwMap, "FlyWheel1",
              BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_6000, DcMotorSimple.Direction.REVERSE,
              DcMotor.RunMode.RUN_WITHOUT_ENCODER,
              BaseMotorWrapper.ENCODER.INTERNAL, DcMotorSimple.Direction.REVERSE,
              1.0, 48.0);
        WheelMotor wheelMotor2 = new WheelMotor(hwMap, "FlyWheel2",
              BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_6000, DcMotorSimple.Direction.FORWARD,
              DcMotor.RunMode.RUN_WITHOUT_ENCODER,
              BaseMotorWrapper.ENCODER.INTERNAL, DcMotorSimple.Direction.FORWARD,
              1.0, 48.0);
        flyWheelSystem = new FlyWheelSystem(hwMap, "FlyWheelSystem", wheelMotor1, wheelMotor2);

        addRobotSystemToList("Drivetrain", drivetrain);
        addRobotSystemToList("FlyWheelSystem", flyWheelSystem);

        robotSystemList.values().forEach(NewCoreRobotSystem::initializeSystem);
    }

    public Pose2D getCurrentPoseVelocity() {
        return ((OctoQuadWrapper) odometry).getCurrentPoseVelocity();
    }

    public OctoQuadFWv3.LocalizerStatus getOQStatus() {
        return ((OctoQuadWrapper) odometry).getLocalizerStatus();
    }

    public boolean isFlyWheelActive() {
        return !(flyWheelSystem.getTargetVelocityRPM() == 0);
    }

    public void turnOffFlyWheel() {
        Msg.log(getClass().getSimpleName(), "turnOffFlyWheel", "Turning off FlyWheel");
        flyWheelSystem.setTargetVelocityRPM(0);
    }

    public FlywheelRC qFlyWheel(int targetRPM) {
        return new FlywheelRC(flyWheelSystem, this::getEHubBulkReadTimeDelta, targetRPM);
    }
}
