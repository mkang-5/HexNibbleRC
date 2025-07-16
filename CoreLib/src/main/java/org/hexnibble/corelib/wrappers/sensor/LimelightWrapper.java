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

  public LimelightWrapper(HardwareMap hwMap, String llName) {
    super(hwMap, Limelight3A.class, llName);

    pythonSettings = new double[] {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
  }

  public void setPollRateHz(int rateHz) {
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

  public boolean pipelineSwitch(int targetPipeline) {
    return sensor.pipelineSwitch(targetPipeline);
  }

  public boolean updatePythonInputs(double[] inputs) {
    return sensor.updatePythonInputs(inputs);
  }

  public LLResult getLatestResult() {
    return sensor.getLatestResult();
  }
}
