package org.hexnibble.corelib.vision;

import static org.hexnibble.corelib.misc.Constants.TAG;

import android.util.Log;
import org.hexnibble.corelib.misc.Timer;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
- stopping streaming or closing the camera at the end of the OpMode does not seem to have any effect
*/

/** This is a base class for EasyOpenCV Pipelines. */
public abstract class BaseOpenCVPipeline extends OpenCvPipeline {
  public enum COLOR {
    BLACK,
    BLUE,
    RED,
    YELLOW
  }

  protected Mat matHSV;
  protected Mat matColorFiltered;
  protected MatOfPoint2f approxContour;

  protected boolean opModeStarted = false;
  protected boolean isPipelineBeingCalled = false;
  protected boolean
      isPipelineActive; // Toggle whether to perform the processing in this pipeline or not
  private Timer pipelineDurationTimer;

  protected double epsilonMultiplicationFactor;

  // OpenCV uses H values from 0 - 180 (i.e., the typical 0 - 360 degree range divided by 2)
  protected Scalar BLACK_MIN_HSV = new Scalar(0, 0, 0);
  protected Scalar BLACK_MAX_HSV = new Scalar(180, 255, 40);
  protected Scalar RED1_MIN_HSV = new Scalar(0, 25, 40); // Red1 is 0-10 deg (converts to 0-5)
  protected Scalar RED1_MAX_HSV = new Scalar(5, 255, 255);
  protected Scalar RED2_MIN_HSV =
      new Scalar(173, 25, 40); // Red2 is 346-360 deg (converts to 173-180)
  protected Scalar RED2_MAX_HSV = new Scalar(180, 255, 255);

  protected Scalar BLUE_MIN_HSV =
      new Scalar(102, 25, 40); // Blue is 205-245 deg (converts to 102.5-122.5)
  protected Scalar BLUE_MAX_HSV = new Scalar(123, 255, 255);
  protected Scalar MAGENTA_MIN_HSV =
      new Scalar(130, 25, 40); // Magenta is 260-345 deg (converts to 130-172.5)
  protected Scalar MAGENTA_MAX_HSV = new Scalar(175, 255, 255);
  protected Scalar CYAN_MIN_HSV =
      new Scalar(87, 25, 40); // Cyan is 175-220 deg (converts to 87.5-110)
  protected Scalar CYAN_MAX_HSV = new Scalar(110, 255, 255);
  protected Scalar YELLOW_MIN_HSV =
      new Scalar(20, 25, 40); // Yellow is 40-65 deg (converts to 20-32.5)
  protected Scalar YELLOW_MAX_HSV = new Scalar(33, 255, 255);

  public BaseOpenCVPipeline() {
    super();

    matHSV = new Mat(); // Matrix to store HSV color image
    matColorFiltered = new Mat(); // Matrix to store color-thresholded image
    approxContour = new MatOfPoint2f();

    epsilonMultiplicationFactor = 0.04;

    isPipelineActive = false;
  }

  public void activatePipeline() {
    isPipelineActive = true;
    Log.i(TAG, "Activating OpenCV pipeline.");
    pipelineDurationTimer = new Timer();
  }

  public void deactivatePipeline() {
    isPipelineActive = false;
    pipelineDurationTimer = null;
    Log.i(TAG, "Deactivating OpenCV pipeline.");
  }

  public boolean isPipelineActive() {
    return isPipelineActive;
  }

  protected long getElapsedTime(Timer.TimerUnit timerUnit) {
    return pipelineDurationTimer.getElapsedTime(timerUnit);
  }

  /** Call this function at the start of the OpMode so that randomization can be saved * */
  public void setOpModeStarted() {
    opModeStarted = true;
    Log.i(TAG, "setOpModeStarted called. Flag is now: " + opModeStarted);
  }

  /**
   * Obtain a region of interest (ROI).
   *
   * @param mat_input input image
   * @param mat_roi image for the roi
   * @param ROIRect Rect containing the x,y coordinates of the upper left corner, and width/height.
   */
  public void getROI(Mat mat_input, Mat mat_roi, Rect ROIRect) {
    // Define ROI rectangle and select it
    mat_roi = mat_input.submat(ROIRect);
  }

  public double getEpsilonMultiplicationFactorForApproximatingContours() {
    return epsilonMultiplicationFactor;
  }

  public void setEpsilonMultiplicationFactorForApproximatingContours(double factor) {
    epsilonMultiplicationFactor = factor;
  }

  /**
   * Converts RGB (the format of the EasyOpenCV webcam stream) to HSV color space to avoid issues
   * with different lighting conditions.
   *
   * @param mat_input
   * @param mat_output
   */
  protected void convertToHSV(Mat mat_input, Mat mat_output) {
    Imgproc.cvtColor(mat_input, mat_output, Imgproc.COLOR_RGB2HSV);
  }

  /**
   * Converts RGB (the format of the EasyOpenCV webcam stream) to grayscale.
   *
   * @param mat_input
   * @param mat_output
   */
  protected void convertToGrayscale(Mat mat_input, Mat mat_output) {
    Imgproc.cvtColor(mat_input, mat_output, Imgproc.COLOR_RGB2GRAY);
  }

  /**
   * Filter by color. Given an HSV image, it is thresholded using the specified color. The result is
   * a black and white image, with the inrange pixels displayed in white.
   */
  protected void colorFilter(COLOR color, Mat mat_input, Mat mat_output) {
    switch (color) {
      case RED:
        Mat red_mat1 = new Mat();
        Mat red_mat2 = new Mat();
        Core.inRange(mat_input, RED1_MIN_HSV, RED1_MAX_HSV, red_mat1);
        Core.inRange(mat_input, RED2_MIN_HSV, RED2_MAX_HSV, red_mat2);
        Core.addWeighted(red_mat1, 1.0, red_mat2, 1.0, 0.0, mat_output);
        red_mat1.release();
        red_mat2.release();
        break;
      case BLUE:
        Core.inRange(mat_input, BLUE_MIN_HSV, BLUE_MAX_HSV, mat_output);
        break;
      case YELLOW:
        Core.inRange(mat_input, YELLOW_MIN_HSV, YELLOW_MAX_HSV, mat_output);
        break;
      case BLACK:
        Core.inRange(mat_input, BLACK_MIN_HSV, BLACK_MAX_HSV, mat_output);
        break;
      default:
        break;
    }
  }

  protected void colorFilter(Scalar minHSV, Scalar maxHSV, Mat mat_input, Mat mat_output) {
    Core.inRange(mat_input, minHSV, maxHSV, mat_output);
  }

  protected void colorFilter(
      Scalar minHSV1,
      Scalar maxHSV1,
      Scalar minHSV2,
      Scalar maxHSV2,
      Mat mat_input,
      Mat mat_output) {
    Mat mat1 = new Mat();
    Mat mat2 = new Mat();

    Core.inRange(mat_input, minHSV1, maxHSV1, mat1);
    Core.inRange(mat_input, minHSV2, maxHSV2, mat2);
    Core.addWeighted(mat1, 1.0, mat2, 1.0, 0.0, mat_output);
  }

  /**
   * Get an approximate contour.
   *
   * @param listOfContours list of found contours
   * @param approxContour
   * @param epsilon_multiplication_factor
   */
  protected void getApproximateContour(
      MatOfPoint listOfContours, MatOfPoint2f approxContour, double epsilon_multiplication_factor) {
    // Convert contour to floating point
    MatOfPoint2f contour_fp = new MatOfPoint2f(); // floating point matrix
    listOfContours.convertTo(contour_fp, CvType.CV_32F);

    // Approximate the contour
    // double epsilon = Imgproc.arcLength(contour_fp, true) * 0.025;
    double epsilon = Imgproc.arcLength(contour_fp, true) * epsilon_multiplication_factor;
    Imgproc.approxPolyDP(contour_fp, approxContour, epsilon, true);
    contour_fp.release();
  }

  /**
   * Detect circles using Hough Circle Transform. This needs a grayscale image. Note the input image
   * will be changed with a median blur.
   *
   * @param mat_input Grayscale image
   * @param mat_circles_list Mat to store any circles found
   * @param minRadius Minimum circle radius (in pixels)
   * @param maxRadius Maximum circle radius (in pixels; if <=0, will ignore radius)
   */
  protected void detectCircles(
      Mat mat_input, Mat mat_circles_list, int param1, int param2, int minRadius, int maxRadius) {
    // Median blur to reduce noise and avoid false circle detection
    Imgproc.medianBlur(mat_input, mat_input, 5);
    Imgproc.HoughCircles(
        mat_input,
        mat_circles_list,
        Imgproc.HOUGH_GRADIENT,
        1.0,
        (double) mat_input.rows() / 16,
        param1,
        param2,
        minRadius,
        maxRadius);
    //        Imgproc.HoughCircles(mat_input, mat_circles_list, Imgproc.HOUGH_GRADIENT, 1.0,
    // (double) mat_input.rows()/16, 200, 100, 5, 500);

  }

  public boolean isPipelineBeingCalled() {
    return isPipelineBeingCalled;
  }

  public void resetIsPipelineBeingCalled() {
    isPipelineBeingCalled = false;
  }

  /**
   * @param inputFrame
   * @return CANNOT BE NULL or the OpMode will crash
   */
  @Override
  public Mat processFrame(Mat inputFrame) {
    isPipelineBeingCalled = true;

    return inputFrame;
  }
}
