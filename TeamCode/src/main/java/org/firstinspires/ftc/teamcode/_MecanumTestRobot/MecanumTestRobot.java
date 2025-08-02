package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.robot.CoreRobot;
import org.hexnibble.corelib.robot.TwoWheelOdometry;
import org.hexnibble.corelib.robot.drivetrain.MecanumDrivetrain;
import org.hexnibble.corelib.robot_system.CoreRobotSystem;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadFWv3;
import org.hexnibble.corelib.wrappers.OctoQuad.OctoQuadWrapper;
import org.hexnibble.corelib.wrappers.motor.BaseMotorWrapper;
import org.hexnibble.corelib.wrappers.sensor.IMUWrapper;

import java.util.Arrays;

public class MecanumTestRobot extends CoreRobot {
    private OctoQuadWrapper oq;
    public MecanumTestRobot(HardwareMap hwMap) {
        super(hwMap, "MecanumTestRobot");
    }

    @Override
    public void initializeSystem() {
//        oq = new OctoQuadWrapper(hwMap, "OctoQuad", 0, OctoQuadFWv3.EncoderDirection.REVERSE,
//                1, OctoQuadFWv3.EncoderDirection.FORWARD,
//                -141.9f, 0.0f);

        IMU = new IMUWrapper(hwMap, IMU_NAME, CH_LOGO_FACING_DIRECTION, CH_USB_FACING_DIRECTION);

        // Create drivetrain object
        drivetrain =
//                new MecanumDrivetrain(hwMap,
//                        "LFMotor", DcMotor.Direction.REVERSE,
//                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,
//                        "RFMotor", DcMotor.Direction.FORWARD,
//                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
//                        "LBMotor", DcMotor.Direction.REVERSE,
//                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,
//                        "RBMotor", DcMotor.Direction.FORWARD,
//                        BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
//                        BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_435, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
//                        1.0, TARGET_POSITION_TOLERANCE, 48.0);

                new MecanumDrivetrain(hwMap,
                  "LFMotor", DcMotor.Direction.REVERSE,
                  BaseMotorWrapper.ENCODER.GO_BILDA_ODOPOD, DcMotorSimple.Direction.FORWARD,
                  "RFMotor", DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.ENCODER.GO_BILDA_ODOPOD, DcMotorSimple.Direction.FORWARD,
                  "LBMotor", DcMotor.Direction.REVERSE,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.REVERSE,
                  "RBMotor", DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.ENCODER.INTERNAL, DcMotor.Direction.FORWARD,
                  BaseMotorWrapper.MOTOR_MODEL.GoBildaYJ_435, DcMotor.RunMode.RUN_WITHOUT_ENCODER,
                  1.0, TARGET_POSITION_TOLERANCE, 48.0);

        addRobotSystemToList("Drivetrain", drivetrain);

        // Create odometry object
        Msg.logIfDebug(className, "initializeSystem", "Creating odometry object.");
        odometry =
              new TwoWheelOdometry(
                    Arrays.asList(
                          new Pose2D(
                                Constants.DRIVETRAIN_LRENCODER_OFFSET_X,
                                Constants.DRIVETRAIN_LRENCODER_OFFSET_Y,
                                Math.PI / 2.0), // Location of LR odometry wheel - x movements
                          new Pose2D(
                                Constants.DRIVETRAIN_FBENCODER_OFFSET_X,
                                Constants.DRIVETRAIN_FBENCODER_OFFSET_Y,
                                0.0) // Location of FB odometry wheel - y movements
                    ),
                    Arrays.asList(
                          drivetrain.getWheelMotorObject(
                                MecanumDrivetrain.WHEEL_MODULE_NAME.LF), // LR odometry wheel (x movements)
                          drivetrain.getWheelMotorObject(
                                MecanumDrivetrain.WHEEL_MODULE_NAME.RF) // FB odometry wheel (y movements)
                    ));

        robotSystemList.values().forEach(CoreRobotSystem::initializeSystem);
    }

//    public Pose2D getCurrentPose() {
//        return oq.getCurrentPose();
//    }

    public Pose2D getCurrentPoseVelocity() {
        return oq.getCurrentPoseVelocity();
    }

    public OctoQuadFWv3.LocalizerStatus getOQStatus() {
        return oq.getLocalizerStatus();
    }

    // region ** IMU Functions **
    /** Reset the IMU heading (yaw) on the Octoquad. */
//    @Override
//    public void resetIMUHeading() {
//        oq.resetIMUHeading();
//    }
//    @Override
//    public double refreshIMUHeading() {
//        return oq.readIMUHeading();
//    }
//
//    @Override
//    public double getStoredIMUHeadingDegrees() {
//        return oq.getStoredIMUHeadingDegrees();
//    }
    // endregion ** IMU Functions **

    @Override
    public void processCommands() {
        boolean sendDriveTrainCommand = false;

        // Update the caches for both the control hub and expansion hub (if it was detected).
        bulkReadControlHub();
//        bulkReadExpansionHub();

        // Update odometry if being used. This needs to be done before any robot commands are processed
        // to have a fresh pose available for use.
        // Also read IMU to get heading if 3-wheel odometry is not being used
        double IMUHeading;
        if (oq != null) {
            oq.readLocalizerData();

            // Calculate heading offset for alliance/field-centric CF if needed
//            if (fieldCentricDrive) {
//                    drivetrain.setCurrentIMUHeading(oq.getStoredIMUHeadingDegrees());
                IMUHeading = oq.getStoredIMUHeadingDegrees();
//            }
        }
        else {
            // Update odometry if being used. This needs to be done before any robot commands are processed
            // to have a fresh pose available for use.
            // Also read IMU to get heading if 3-wheel odometry is not being used
            if (odometry != null) {
                if (odometry instanceof TwoWheelOdometry) {
                    IMUHeading = refreshIMUHeading();
                    odometry.updateOdometry(IMUHeading); // Update cumulative translation movements
                }
                else {
                    odometry.updateOdometry(Double.NaN); // Update cumulative translation movements
                    IMUHeading = Math.toDegrees(odometry.getPoseEstimate().heading);
                }
            }
            else {
                IMUHeading = refreshIMUHeading();
            }
        }

        // Send updated IMU heading to drivetrain
        if (drivetrain != null) {
            // Calculate heading offset for alliance/field-centric CF if needed
            if (fieldCentricDrive) {
                drivetrain.setCurrentIMUHeading(IMUHeading);
            }
        }

        robotSystemList.values().forEach(CoreRobotSystem::processCommands);
        super.processCommands();
//        drivetrain.processCommands();
    }
}
