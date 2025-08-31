package org.hexnibble.corelib.wrappers.OctoQuad;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.robot.OdometryIface;
import org.hexnibble.corelib.wrappers.sensor.IMUIface;

import java.util.ArrayList;

public class OctoQuadWrapper implements IMUIface, OdometryIface {
    private double currentIMUHeadingDegrees = 0.0;
    OctoQuadFWv3 oq;

    int LR_odoWheelPort;
    int FB_odoWheelPort;

    // Data structure which will store the localizer data read from the OctoQuad
    OctoQuadFWv3.LocalizerDataBlock localizerData;
    OctoQuadFWv3.EncoderDataBlock encoderData;

//    private static final float goBILDA_SWINGARM_POD_COUNTS_PER_MM = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
//    private static final float goBILDA_4_BAR_POD_COUNTS_PER_MM = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod

    private static final float goBILDA_4_BAR_POD_COUNTS_PER_MM_X = 19.950f;     // Counts/mm in our X direction
    private static final float goBILDA_4_BAR_POD_COUNTS_PER_MM_Y = 20.0585f;    // Counts/mm in our Y direction

    private static final float IMUHeadingScalar = 1.0f;
    private static final int LocalizerVelocityInterval_ms = 10;
    private static final int EncoderVelocityInterval_ms = 10;

    private final String className = getClass().getSimpleName();

    /**
     * Constructor for OctoQuadWrapper.
     * Although the OctoQuad uses a different robot-centric coordinate system than we do
     * (where +X is forward, +Y is left, and +Heading is CCW spin), use values relative to our
     * robot-centric coordinate system for this constructor. They will be appropriately converted.
     *
     * @param LR_odoWheelPort OQ Port for LR encoder (0 - 7)
     * @param LR_odoWheelDirection Direction of the LR encoder (+ is positive X/horizontal axis)
     */
    public OctoQuadWrapper(HardwareMap hwMap, String oqDeviceName,
                           int LR_odoWheelPort, OctoQuadFWv3.EncoderDirection LR_odoWheelDirection,
                           int FB_odoWheelPort, OctoQuadFWv3.EncoderDirection FB_odoWheelDirection,
                           float tcpOffsetX_mm, float tcpOffsetY_mm) {
        oq = hwMap.get(OctoQuadFWv3.class, oqDeviceName);
        oq.resetEverything();

        oq.setChannelBankConfig(OctoQuadFWv3.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);

        localizerData = new OctoQuadFWv3.LocalizerDataBlock();
        encoderData = new OctoQuadFWv3.EncoderDataBlock();

        this.LR_odoWheelPort = LR_odoWheelPort;     // For LR directions, the OctoQuad uses +Y to the left.
        this.FB_odoWheelPort = FB_odoWheelPort;     // For FB directions, the OctoQuad uses +X forward.

        // Configure a whole bunch of parameters for the absolute localizer
        // --> Read the quick start guide for an explanation of these!!
        // IMPORTANT: these parameter changes will not take effect until the localizer is reset!

        // For LR directions, the OctoQuad uses +Y to the left.
        if (LR_odoWheelDirection == OctoQuadFWv3.EncoderDirection.FORWARD) {
            oq.setSingleEncoderDirection(LR_odoWheelPort, OctoQuadFWv3.EncoderDirection.REVERSE);
        }
        else {
            oq.setSingleEncoderDirection(LR_odoWheelPort, OctoQuadFWv3.EncoderDirection.FORWARD);
        }
//        oq.setLocalizerPortY(LR_odoWheelPort);
//        oq.setLocalizerCountsPerMM_Y(goBILDA_4_BAR_POD_COUNTS_PER_MM_X);
//        oq.setLocalizerTcpOffsetMM_Y(-tcpOffsetX_mm);

        // For FB directions, the OctoQuad uses +X forward.
        oq.setSingleEncoderDirection(FB_odoWheelPort, FB_odoWheelDirection);
//        oq.setLocalizerPortX(FB_odoWheelPort);
//        oq.setLocalizerCountsPerMM_X(goBILDA_4_BAR_POD_COUNTS_PER_MM_Y);
//        oq.setLocalizerTcpOffsetMM_X(tcpOffsetY_mm);

        oq.setAllLocalizerParameters(
              FB_odoWheelPort, LR_odoWheelPort,
              goBILDA_4_BAR_POD_COUNTS_PER_MM_Y, goBILDA_4_BAR_POD_COUNTS_PER_MM_X,
              tcpOffsetY_mm, -tcpOffsetX_mm,
              IMUHeadingScalar, LocalizerVelocityInterval_ms);

//        oq.setLocalizerImuHeadingScalar(1.0f);
//        oq.setLocalizerVelocityIntervalMS(15);

        oq.setSingleVelocitySampleInterval(2, EncoderVelocityInterval_ms);
        oq.setSingleVelocitySampleInterval(3, EncoderVelocityInterval_ms);

        oq.setI2cRecoveryMode(OctoQuadFWv3.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        // Reset the localizer pose to (0, 0, 0) and calibrate IMU, using the settings above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        oq.resetLocalizerAndCalibrateIMU();
    }

    public void saveParametersToFlash() {
        oq.saveParametersToFlash();
    }

    public void setPortToDistanceSensor(int port) {
        oq.setSingleChannelPulseWidthParams(port, 1, 1850);
    }

    // region IMUIface Functions
    @Override
    public void resetIMUHeading() {
        resetEncodersAndPose();
    }

    /**
     * This function will not refresh the IMU for the OctoQuad. It will only read the stored heading.
     * To refresh, use updateOdometry()
     * @return Stored IMU heading (degrees)
     */
    @Override
    public double refreshIMUHeading() {
        return getIMUHeadingDegrees();
    }

    @Override
    public double getStoredIMUHeadingDegrees() {
        return getIMUHeadingDegrees();
    }
    // endregion IMUIface Functions


    /**
     * Reset odometry wheel encoders and pose
     */
    @Override
    public void resetEncodersAndPose() {
        oq.resetSinglePosition(LR_odoWheelPort);
        oq.resetSinglePosition(FB_odoWheelPort);
        oq.resetLocalizerAndCalibrateIMU();
    }

    /**
     * Set current pose (alliance-centric CF).
     *
     * @param newPose Alliance-centric X, Y coordinates (mm) and alliance-centric heading (radians)
     */
    @Override
    public void setPoseEstimate(Pose2D newPose) {
        // Convert to the OctoQuad coordinate system (+x = forward; +y = left)
        oq.setLocalizerPose((int) newPose.y, -(int) newPose.x, (float) newPose.heading);
    }

    /**
     * Read both localizer (pose, including IMU) and encoder data.
     */
    @Override
    public void updateOdometry(double IMUHeadingDegrees) {
        OctoQuadFWv3.LocalizerDataBlock tempLocalizerData = new OctoQuadFWv3.LocalizerDataBlock();
        OctoQuadFWv3.EncoderDataBlock tempEncoderData = new OctoQuadFWv3.EncoderDataBlock();

        oq.readLocalizerDataAndAllEncoderData(tempLocalizerData, tempEncoderData);
        if (tempLocalizerData.crcOk) {
            localizerData = tempLocalizerData;
            currentIMUHeadingDegrees = Math.toDegrees(localizerData.heading_rad);
        }
        else {
            Msg.log(className, "updateOdometry", "OQ CRC error - localizer data");
        }

        if (tempEncoderData.crcOk) {
            encoderData = tempEncoderData;
        }
        else {
            Msg.log(className, "updateOdometry", "OQ CRC error - encoder data");
        }
    }

    /**
     * Get the current alliance-centric pose.
     *
     * @return Alliance CF pose. Heading is IMU-style in radians.
     */
    @Override
    public Pose2D getCurrentPose() {
        // Convert OctoQuad pose to our pose
        return new Pose2D(-localizerData.posY_mm, localizerData.posX_mm, localizerData.heading_rad);
    }

    /**
     * Get the current alliance-centric IMU heading.
     *
     * @return Alliance CF IMU heading (degrees).
     */
    @Override
    public double getIMUHeadingDegrees() {
        return currentIMUHeadingDegrees;
    }

    /**
     * Obtain encoder counts as a list using the most recently read data.
     * @return List of encoder counts. LR encoder is index 0. FB encoder is index 1.
     */
    @Override
    public ArrayList<Integer> getOdometryEncoderCounts() {
        ArrayList<Integer> positions = new ArrayList<>(2);
        positions.add(encoderData.positions[LR_odoWheelPort]);
        positions.add(encoderData.positions[FB_odoWheelPort]);
        return positions;
    }

    /**
     * Obtain encoder positions (mm) as a list using the most recently read data.
     * @return List of encoder positions (mm). LR encoder is index 0. FB encoder is index 1.
     */
    public ArrayList<Double> getOdometryEncoderPositions_mm() {
        ArrayList<Double> positions = new ArrayList<>(2);
        positions.add(encoderData.positions[LR_odoWheelPort] / (double) goBILDA_4_BAR_POD_COUNTS_PER_MM_X);
        positions.add(encoderData.positions[FB_odoWheelPort] / (double) goBILDA_4_BAR_POD_COUNTS_PER_MM_Y);
        return positions;
    }

    /**
     * Obtain position information for the specified port.
     * For a quadrature encoder port, this retrieves the encoder count.
     * For a PWM port, this retrieves the pulse width (us)
     * @param portNumber Port number (0 - 7) for the desired encoder
     * @return Encoder count or pulse width (us)
     */
    public int getPositionInformation(int portNumber) {
        return encoderData.positions[portNumber];
    }

    /**
     * Obtain encoder velocity for the specified encoder.
     * @param portNumber Port number (0 - 7) for the desired encoder
     * @return Encoder velocity
     */
    public int getEncoderVelocity(int portNumber) {
        return encoderData.velocities[portNumber];
    }

    public void setLocalizerHeading(float headingRad) {
        oq.setLocalizerHeading(headingRad);
    }

    /**
     * Read only localizer data (pose, including IMU).
     */
    public void readLocalizerData() {
        OctoQuadFWv3.LocalizerDataBlock tempLocalizerData = new OctoQuadFWv3.LocalizerDataBlock();
        oq.readLocalizerData(tempLocalizerData);

        if (tempLocalizerData.crcOk) {
            localizerData = tempLocalizerData;
            currentIMUHeadingDegrees = Math.toDegrees(localizerData.heading_rad);
        }
    }

    public boolean isLocalizerDataCrcOk() {
        return localizerData.crcOk;
    }

    public Pose2D getCurrentPoseVelocity() {
        return new Pose2D(-localizerData.velY_mmS, localizerData.velX_mmS, localizerData.velHeading_radS);
    }

    /**
     * This function can be called each OpMode loop to read only encoder info from the OctoQuad
     */
    public void readEncoderData() {
        OctoQuadFWv3.EncoderDataBlock tempEncoderData = new OctoQuadFWv3.EncoderDataBlock();
        oq.readAllEncoderData(tempEncoderData);

        if (tempEncoderData.crcOk) {
            encoderData = tempEncoderData;
        }
    }

    public boolean isEncoderDataCrcOk() {
        return encoderData.crcOk;
    }

    public OctoQuadFWv3.LocalizerStatus getLocalizerStatus() {
        return oq.getLocalizerStatus();
    }
}





//    public void readPorts() {
//        OctoQuadFWv3.LocalizerDataBlock localizerData = new OctoQuadFWv3.LocalizerDataBlock();
//        OctoQuadFWv3.EncoderDataBlock encoderData = new OctoQuadFWv3.EncoderDataBlock();
//        m_octoQuad.readLocalizerDataAndAllEncoderData(localizerData, encoderData);
//
//        if (localizerData.crcOk) {
//            m_localizerData = localizerData;
//        }
//        if (encoderData.crcOk) {
//            m_encoderData = encoderData;
//        }
//    }
//}
