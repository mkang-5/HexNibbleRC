package org.hexnibble.corelib.vision;

import static org.hexnibble.corelib.misc.Constants.TAG;

import android.util.Log;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCVCameraWrapper {
  protected OpenCvWebcam camera;
  protected boolean isCameraOpened = false;

  private final int width;
  private final int height;
  private final OpenCvCameraRotation cameraRotation;
  private final OpenCvWebcam.StreamFormat streamFormat;

  public OpenCVCameraWrapper(
      HardwareMap hwMap,
      String cameraName,
      int width,
      int height,
      OpenCvCameraRotation cameraRotation,
      OpenCvWebcam.StreamFormat streamFormat) {
    this.width = width;
    this.height = height;
    this.cameraRotation = cameraRotation;
    this.streamFormat = streamFormat;

    // Obtain view ID for camera monitor
    int cameraMonitorViewId =
        hwMap
            .appContext
            .getResources()
            .getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());

    // Create a webcam instance with live preview
    camera =
        OpenCvCameraFactory.getInstance()
            .createWebcam(hwMap.get(WebcamName.class, cameraName), cameraMonitorViewId);
  }

  /**
   * Attaches the specified pipeline and opens the camera.
   *
   * @param openCVPipeline
   */
  public void initializeCamera(OpenCvPipeline openCVPipeline) {
    attachPipeline(openCVPipeline);

    // Open the camera device asynchronously
    camera.openCameraDeviceAsync(
        new OpenCvCamera.AsyncCameraOpenListener() {
          @Override
          public void onOpened() {
            isCameraOpened = true;

            camera.startStreaming(width, height, cameraRotation, streamFormat);
            //                pauseViewport();
          }

          @Override
          public void onError(int errorCode) {
            isCameraOpened = false;
          }
        });
  }

  public void attachPipeline(OpenCvPipeline openCVPipeline) {
    // Specify an image processing pipeline
    camera.setPipeline(openCVPipeline);
  }

  /** If the webcam object exists, stop streaming and close the camera device. */
  public void closeCamera() {
    Log.i(TAG, "OpenCVCameraWrapper.j: closeCamera called.");
    // Stop streaming the camera and close the camera device
    if (camera != null) {
      camera.closeCameraDeviceAsync(
          new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
              isCameraOpened = false;
              Log.i(TAG, "OpenCVCameraWrapper.j: Camera closed.");
            }
          });
    }
  }

  public boolean isCameraOpened() {
    return isCameraOpened;
  }

  public void startStreaming() {
    if (camera != null) camera.startStreaming(width, height, cameraRotation, streamFormat);
  }

  /**
   * Stopping streaming will stop the HDMI feed/live view AND the pipeline processing (i.e. calls to
   * processFrame).
   */
  public void stopStreaming() {
    if (camera != null) camera.stopStreaming();
  }

  /**
   * Pause the viewport preview (i.e. the HDMI feed/live view). This does not stop pipeline
   * processing (i.e. calls to processFrame).
   */
  public void pauseViewport() {
    if (camera != null) camera.pauseViewport();
  }

  public void resumeViewport() {
    if (camera != null) camera.resumeViewport();
  }
}
