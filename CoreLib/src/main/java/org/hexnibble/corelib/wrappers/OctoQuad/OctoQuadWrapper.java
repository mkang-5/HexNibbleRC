package org.hexnibble.corelib.wrappers.OctoQuad;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
//import org.hexnibble.corelib.wrappers.sensor.IMUIface;

//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class OctoQuadWrapper implements IMUIface {
public class OctoQuadWrapper {
    private double currentIMUHeadingDegrees = 0.0;
    OctoQuadFWv3 oq;

    // Data structure which will store the localizer data read from the OctoQuad
    OctoQuadFWv3.LocalizerDataBlock localizerData;
    OctoQuadFWv3.EncoderDataBlock encoderData;

    private static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    private static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod

    /**
     * Note that this is different from
     * our coordinate system.
     */
    public OctoQuadWrapper(HardwareMap hwMap, String oqDeviceName,
                           int LR_odoWheelPort, OctoQuadFWv3.EncoderDirection LR_odoWheelDirection,
                           int FB_odoWheelPort, OctoQuadFWv3.EncoderDirection FB_odoWheelDirection,
                           float tcpOffsetX_mm, float tcpOffsetY_mm) {
        oq = hwMap.get(OctoQuadFWv3.class, oqDeviceName);
        localizerData = new OctoQuadFWv3.LocalizerDataBlock();
        encoderData = new OctoQuadFWv3.EncoderDataBlock();

        oq.resetEverything();

        // The OctoQuad uses +X to represent forward and +Y to the left.
        // Configure a whole bunch of parameters for the absolute localizer
        // --> Read the quick start guide for an explanation of these!!
        // IMPORTANT: these parameter changes will not take effect until
        // the localizer is reset!
        oq.setSingleEncoderDirection(LR_odoWheelPort, LR_odoWheelDirection);
        oq.setLocalizerPortY(LR_odoWheelPort);

        oq.setSingleEncoderDirection(FB_odoWheelPort, FB_odoWheelDirection);
        oq.setLocalizerPortX(FB_odoWheelPort);

        oq.setLocalizerCountsPerMM_X(goBILDA_4_BAR_POD);
        oq.setLocalizerCountsPerMM_Y(goBILDA_4_BAR_POD);
        oq.setLocalizerTcpOffsetMM_X(tcpOffsetY_mm);
        oq.setLocalizerTcpOffsetMM_Y(-tcpOffsetX_mm);
        oq.setLocalizerImuHeadingScalar(1.0f);

        oq.setLocalizerVelocityIntervalMS(20);
        oq.setI2cRecoveryMode(OctoQuadFWv3.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        // Reset the localizer pose to (0, 0, 0) and calibrate IMU, using the settings above.
        // This function will NOT block until calibration of the IMU is complete -
        // for that you need to look at the status returned by getLocalizerStatus()
        oq.resetLocalizerAndCalibrateIMU();
    }

    /**
     * Set the localizer position (alliance-centric CF).
     * @param X_mm
     * @param Y_mm
     * @param headingRad
     */
    public void setLocalizerPose(int X_mm, int Y_mm, float headingRad) {
        // Convert to the OctoQuad coordinate system (+x = forward; +y = left)
        oq.setLocalizerPose(Y_mm, -X_mm, headingRad);
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

    /**
     *
     * @return Alliance CF pose. Heading is IMU-style in radians.
     */
    public Pose2D getCurrentPose() {
        return new Pose2D(-localizerData.posY_mm, localizerData.posX_mm, localizerData.heading_rad);
    }

    /**
     * This function should be called each OpMode loop to read pose info from the OctoQuad
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

    /**
     * Read both localizer (pose, including IMU) and encoder data.
     */
    public void readLocalizerAndEncoderData() {
        OctoQuadFWv3.LocalizerDataBlock tempLocalizerData = new OctoQuadFWv3.LocalizerDataBlock();
        OctoQuadFWv3.EncoderDataBlock tempEncoderData = new OctoQuadFWv3.EncoderDataBlock();

        oq.readLocalizerDataAndAllEncoderData(tempLocalizerData, tempEncoderData);
        if (tempLocalizerData.crcOk) {
            localizerData = tempLocalizerData;
            currentIMUHeadingDegrees = Math.toDegrees(localizerData.heading_rad);
        }
        if (tempEncoderData.crcOk) {
            encoderData = tempEncoderData;
        }
    }


    public void resetIMUHeading() {
        oq.resetLocalizerAndCalibrateIMU();
    }

    /**
     * This is similar to readLocalizerAndEncoderData but returns the IMU heading.
     * @return IMU Heading (degrees)
     */

    public double readIMUHeading() {
        readLocalizerAndEncoderData();

        return currentIMUHeadingDegrees;
    }


    public double getStoredIMUHeadingDegrees() {
        return currentIMUHeadingDegrees;
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
