package org.hexnibble.corelib.motion.pid;

public class PIDController {
  protected boolean targetStateUpdated = true; // Flag whether the original target state was updated
  protected double targetTolerance;

  protected final double Ks;
  protected final double Kp; // Proportional term
  protected final double Ki;
  protected final double Kd;

  protected double sumOfITerms = 0.0;
  protected final int NUM_ERROR_VALUES_TO_STORE = 50;
  protected double[] arrayOfErrors = new double[NUM_ERROR_VALUES_TO_STORE];
  protected int numStoredErrorValues = 0;
//  protected int lastErrorSign;

//  protected double absPreviousError = 0.0;
  protected double previousError = 0.0;
  protected long previousSystemTime_ms = 0L;

  protected boolean activateTargetCounter = false;
  protected int atTargetCounter = 0;

  private final int TARGET_COUNTER_THRESHOLD_FOR_COMPLETENESS = 10;

  /*
      /**
       * # For a basic drivetrain (with nothing on top) on floor, simple working values are: Ks = 0.05, Kp = 0.001, Ki = 0.0, Kd = 0.0
  //     * @param targetState
       * @param targetTolerance
       * @param Ks Ks is a static term that specifies a minimum power.
       * @param Kp Kp is the proportional term. It is directly proportional to the error (Kp * error). Higher values will result in greater moves towards 0 error.
       * @param Ki Ki is the integral term. It is directly proportional to the sum of all errors (or a finite lookback period). It addresses nonlinear effects (e.g. friction) by allowing cumulative errors to overcome a constant disturbance.
       * @param Kd Kd is the derivative term. It is directly proportional to the rate of change of error. It is multiplied by the difference between the last error and the current error (also divided by time elapsed).
       */

  /*    public PIDController(double targetState, double targetTolerance, double Ks, double Kp, double Ki, double Kd) {
          this.targetState = targetState;
          this.targetStateUpdated = true;
          this.targetTolerance = targetTolerance;
          this.Ks = Ks;
          this.Kp = Kp;
          this.Ki = Ki;
          this.Kd = Kd;
  }
  */

  /**
   * For a basic drivetrain (with nothing on top) on floor, simple working values are: Ks = 0.05, Kp
   * = 0.001, Ki = 0.0, Kd = 0.0
   *
   * @param targetTolerance
   * @param Ks Ks is a static term that specifies a minimum power.
   * @param Kp Kp is the proportional term. It is directly proportional to the error (Kp * error).
   *     Higher values will result in greater moves towards 0 error.
   * @param Ki Ki is the integral term. It is directly proportional to the sum of all errors (or a
   *     finite lookback period). It addresses nonlinear effects (e.g. friction) by allowing
   *     cumulative errors to overcome a constant disturbance.
   * @param Kd Kd is the derivative term. It is directly proportional to the rate of change of
   *     error. It is multiplied by the difference between the last error and the current error
   *     (also divided by time elapsed).
   */
  public PIDController(double Ks, double Kp, double Ki, double Kd, double targetTolerance) {
    this.targetTolerance = targetTolerance;
    this.Ks = Ks;
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
    this.activateTargetCounter = true;
  }

  public PIDController(PIDSettings settings, double targetTolerance) {
    this(settings.Ks, settings.Kp, settings.Ki, settings.Kd, targetTolerance);
  }

  public void setTargetTolerance(double targetTolerance) {
    this.targetTolerance = targetTolerance;
  }

  //    public void activateTargetCounter() {
  //        activateTargetCounter = true;
  //    }

  public void reset() {
    resetTargetCounter();
    resetErrorHistories();
  }

  public void resetTargetCounter() {
    atTargetCounter = 0;
  }

  public boolean isCommandComplete() {
    return (atTargetCounter > TARGET_COUNTER_THRESHOLD_FOR_COMPLETENESS);
  }

  public void resetErrorHistories() {
    sumOfITerms = 0.0;
    arrayOfErrors = new double[NUM_ERROR_VALUES_TO_STORE];
    numStoredErrorValues = 0;
//    lastErrorSign = 0;

//    absPreviousError = 0.0;
  }

  public double calculateNewControlValue(double currentError) {
    double absCurrentError = Math.abs(currentError);
    int errorSign = (int) (absCurrentError / currentError);
    long currentSystemTime_ms = System.currentTimeMillis();
    long deltaTime = currentSystemTime_ms - previousSystemTime_ms;

    double P = Kp * currentError;

    double I = 0.0;
    if (Ki != 0.0) {
      // If the error sign is different from previous, then empty the buffer array
//      if (lastErrorSign != errorSign) {
//        arrayOfErrors = new double[NUM_ERROR_VALUES_TO_STORE];
//        numStoredErrorValues = 0;
//        sumOfITerms = 0.0;
//      }

      // Calculate the integral term, I.
      // There is a maximum number of values set by NUM_ERROR_VALUES_TO_STORE to keep this term
      // more relevant in the setting of rapidly decreasing error as we near the setpoint.
      // Store error value if there is still space in the array. Otherwise, remove the oldest
      // value before storing the current one.
      if (numStoredErrorValues < NUM_ERROR_VALUES_TO_STORE) {
        arrayOfErrors[numStoredErrorValues] = currentError;
        numStoredErrorValues++;
      }
      else { // Remove the earliest item in the array
        sumOfITerms -= arrayOfErrors[0];
        System.arraycopy(arrayOfErrors, 1, arrayOfErrors, 0, (NUM_ERROR_VALUES_TO_STORE - 1));
        arrayOfErrors[numStoredErrorValues - 1] = currentError;
      }

      sumOfITerms += currentError;

      I = Ki * sumOfITerms;
    }

    // Calculate the derivative term, D
    // Since this is a fairly short calculation, don't bother checking whether Kd is zero since
    // that branching is likely to slow things down
    double D = Kd * (currentError - previousError) / (double) deltaTime;

//    if (Kd != 0.0) {
//      double deltaAbsError = absCurrentError - absPreviousError;
//      D = Kd * deltaAbsError / (double) deltaTime;
//    } else {
//      D = 0.0;
//    }

//    absPreviousError = absCurrentError;
    previousError = currentError;
    previousSystemTime_ms = currentSystemTime_ms;

    // Check if the target counter has been activated
    if (activateTargetCounter) {
      // If the current error is within the threshold, increase the counter
      if (absCurrentError <= targetTolerance) {
        atTargetCounter++;
      } else {
        atTargetCounter = 0;
      }
    }

//    lastErrorSign = errorSign;

//    return (Ks * errorSign) + P + I + D;
    return Ks + P + I + D;
  }
}
