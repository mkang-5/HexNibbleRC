package org.hexnibble.corelib.wrappers.sensor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightWrapper extends CoreSensorWrapper<Limelight3A> {
  public enum TARGET_COLOR {
    YELLOW,
    RED,
    BLUE
  }

  private TARGET_COLOR targetColor;
  public static final double YELLOW = 1.0;
  public static final double RED = 2.0;
  public static double BLUE = 3.0;

  //    private final Limelight3A limelight;
  private final double[] pythonSettings;

  private final int DEFAULT_POLL_RATE_HZ = 100;
  private int pollRateHz;

  public LimelightWrapper(HardwareMap hwMap, String llName) {
    super(hwMap, Limelight3A.class, llName);

    pythonSettings = new double[] {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    this.pollRateHz = DEFAULT_POLL_RATE_HZ;
    sensor.setPollRateHz(DEFAULT_POLL_RATE_HZ);
  }

  public void reset() {
    sensor.stop();
    sensor.setPollRateHz(pollRateHz);
  }

  public void setPollRateHz(int rateHz) {
    pollRateHz = rateHz;
    sensor.setPollRateHz(rateHz);
  }

  public void setTargetColor(TARGET_COLOR targetColor) {
    this.targetColor = targetColor;
    pythonSettings[0] =
        switch (targetColor) {
          case YELLOW -> YELLOW;
          case RED -> RED;
          case BLUE -> BLUE;
        };

    sensor.updatePythonInputs(pythonSettings);
  }

  /**
   * Set pipeline: 0 = Red Sample 1 = Blue Sample 2 = Yellow Sample
   *
   * @param targetPipeline
   * @return True if pipeline successfully set
   */
  public boolean setPipeline(int targetPipeline) {
    return sensor.pipelineSwitch(targetPipeline);
  }

  public boolean updatePythonInputs(double[] inputs) {
    return sensor.updatePythonInputs(inputs);
  }

  public LLResult getLatestResult() {
    return sensor.getLatestResult();
  }

  /** Start polling of Limelight data */
  public void start() {
    sensor.start();
  }

  public void stop() {
    sensor.stop();
  }

  public void close() {
    sensor.close();
  }

  public boolean isDisconnected() {
    return sensor == null || !sensor.isConnected();
  }

  public boolean isRunning() {
    return sensor != null && sensor.isRunning();
  }

}
