//package org.hexnibble.corelib.opmodes.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.hexnibble.corelib.commands.DrivetrainRC;
//import org.hexnibble.corelib.misc.ConfigFile;
//import org.hexnibble.corelib.misc.Constants;
//import org.hexnibble.corelib.misc.Field;
//import org.hexnibble.corelib.misc.Pose2D;
//import org.hexnibble.corelib.motion.pid.PIDSettings;
//import org.hexnibble.corelib.opmodes.CoreTeleOpMode;
//import org.hexnibble.corelib.wrappers.controller.ButtonWrapper;
//import org.hexnibble.corelib.wrappers.controller.ControllerWrapper;
//
///** This is a linear driver-controlled mode for drivetrain PID tuning. */
//@TeleOp(name = "PID Tuner Test", group = "Test OpModes")
//@Disabled
//public class DrivetrainPIDTestOpMode extends CoreTeleOpMode {
//  // For a basic drivetrain (with nothing on top) on floor, simple working values are: Ks = 0.05, Kp
//  // = 0.001, Ki = 0.0, Kd = 0.0
//  //      Ks is a static term that specifies a minimum power.
//  //      Kp is the proportional term. It is directly proportional to the error (Kp * error). Higher
//  // values will result in greater moves towards 0 error.
//  //          Kp cannot be 0.
//  //      Ki is the integral term. It is directly proportional to the sum of all errors (for a
//  // finite lookback period). It addresses nonlinear effects (e.g. friction) by allowing cumulative
//  // errors to overcome a constant disturbance.
//  //      Kd is the derivative term. It is directly proportional to the rate of change of error. It
//  // is multiplied by the difference between the last error and the current error (also divided by
//  // time elapsed). It works to dampen changes.
//  private final double xyPID_Ks = 0.001;
//  private final double xyPID_Kp = 0.007;
//  private final double xyPID_Ki = 0.0;
//  private final double xyPID_Kd = 0.0;
//
//  private final double rotPID_Ks = 0.05;
//  private final double rotPID_Kp = 1.15;
//  private final double rotPID_Ki = 0.0;
//  private final double rotPID_Kd = 0.0;
//
//  private final double tileWidth = Field.FIELD_WIDTH_MM / 6.0;
//
//  PIDSettings xyPIDSettings;
//  PIDSettings rotationPIDSettings;
//
//  enum MOVEMENT_TYPE {
//    TRANSLATION,
//    ROTATION
//  }
//
//  MOVEMENT_TYPE movementType;
//
//  public DrivetrainPIDTestOpMode() {
//    super(new ConfigFile(configFileName), new Constants());
//
//    xyPIDSettings = new PIDSettings(xyPID_Ks, xyPID_Kp, xyPID_Ki, xyPID_Kd);
//
//    rotationPIDSettings = new PIDSettings(rotPID_Ks, rotPID_Kp, rotPID_Ki, rotPID_Kd);
//
//    movementType = MOVEMENT_TYPE.TRANSLATION;
//  }
//
//  protected void processController1() {
//    //        super.processController1();
//
//    // Shift Buttons - Share and Option
//    boolean isShareShiftPressed = controller1.isShareShiftPressed();
//    boolean isOptionShiftPressed = controller1.isOptionShiftPressed();
//
//    if (controller1.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.touchpad)) {
//      if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//        movementType = MOVEMENT_TYPE.ROTATION;
//      } else {
//        movementType = MOVEMENT_TYPE.TRANSLATION;
//      }
//    }
//    Pose2D deltaPose;
//    if (controller1.isButtonNewlyPressed(ControllerWrapper.BUTTON_NAME.dpad_left)) {
//      if (isOptionShiftPressed) {
//        deltaPose = new Pose2D(-tileWidth, tileWidth, Math.toRadians(45.0));
//        qDeltaDrivetrainRC(
//            "Joystick_QuantumNWRotate", deltaPose, null, xyPIDSettings, rotationPIDSettings);
//      } else {
//        deltaPose = new Pose2D(-tileWidth * 2, 0.0, 0.0);
//        qDeltaDrivetrainRC(
//            "Joystick_QuantumLeft", deltaPose, null, xyPIDSettings, rotationPIDSettings);
//      }
//    } else if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_down)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      deltaPose = new Pose2D(0.0, -tileWidth * 2, Math.toRadians(0.0));
//      qDeltaDrivetrainRC(
//          "Joystick_QuantumReverse", deltaPose, null, xyPIDSettings, rotationPIDSettings);
//    } else if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_right)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      if (isOptionShiftPressed) {
//        deltaPose = new Pose2D(tileWidth, -tileWidth, Math.toRadians(-45.0));
//        qDeltaDrivetrainRC(
//            "Joystick_QuantumNWRotate", deltaPose, null, xyPIDSettings, rotationPIDSettings);
//      } else {
//        deltaPose = new Pose2D(tileWidth * 2, 0.0, 0.0);
//        qDeltaDrivetrainRC(
//            "Joystick_QuantumRight", deltaPose, null, xyPIDSettings, rotationPIDSettings);
//      }
//    } else if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.dpad_up)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      deltaPose = new Pose2D(0.0, tileWidth * 2, Math.toRadians(0.0));
//      qDeltaDrivetrainRC(
//          "Joystick_QuantumForward", deltaPose, null, xyPIDSettings, rotationPIDSettings);
//    }
//
//    if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.left_bumper)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      deltaPose = new Pose2D(0.0, 0.0, Math.toRadians(135.0));
//      qDeltaDrivetrainRC(
//          "Joystick_CCWTurn",
//          deltaPose,
//          DrivetrainRC.ROTATION_DIRECTION.COUNTERCLOCKWISE,
//          xyPIDSettings,
//          rotationPIDSettings);
//    } else if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.right_bumper)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      deltaPose = new Pose2D(0.0, 0.0, Math.toRadians(-135.0));
//      qDeltaDrivetrainRC(
//          "Joystick_CWTurn",
//          deltaPose,
//          DrivetrainRC.ROTATION_DIRECTION.CLOCKWISE,
//          xyPIDSettings,
//          rotationPIDSettings);
//    }
//
//    if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.square)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      if (isOptionShiftPressed) {
//        robot.resetIMUHeading();
//      } else if (isShareShiftPressed) {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Kp = Math.max(xyPIDSettings.Kp - 0.001, 0.0);
//        } else {
//          rotationPIDSettings.Kp = Math.max(rotationPIDSettings.Kp - 0.01, 0.0);
//        }
//      } else {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Kp = xyPIDSettings.Kp + 0.001;
//        } else {
//          rotationPIDSettings.Kp = rotationPIDSettings.Kp + 0.01;
//        }
//      }
//    }
//
//    if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.triangle)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      if (isShareShiftPressed) {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Ki = Math.max(xyPIDSettings.Ki - 0.00001, 0.0);
//        } else {
//          rotationPIDSettings.Ki = Math.max(rotationPIDSettings.Ki - 0.00001, 0.0);
//        }
//      } else {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Ki = xyPIDSettings.Ki + 0.00001;
//        } else {
//          rotationPIDSettings.Ki = rotationPIDSettings.Ki + 0.00001;
//        }
//      }
//    }
//
//    if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.circle)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      if (isShareShiftPressed) {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Kd = Math.max(xyPIDSettings.Kd - 0.001, 0.0);
//        } else {
//          rotationPIDSettings.Kd = Math.max(rotationPIDSettings.Kd - 0.001, 0.0);
//        }
//      } else {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Kd = xyPIDSettings.Kd + 0.001;
//        } else {
//          rotationPIDSettings.Kd = rotationPIDSettings.Kd + 0.001;
//        }
//      }
//    }
//
//    if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.cross)
//        == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//      if (isShareShiftPressed) {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Ks = Math.max(xyPIDSettings.Ks - 0.001, 0.0);
//        } else {
//          rotationPIDSettings.Ks = Math.max(rotationPIDSettings.Ks - 0.01, 0.0);
//        }
//      } else {
//        if (movementType == MOVEMENT_TYPE.TRANSLATION) {
//          xyPIDSettings.Ks = xyPIDSettings.Ks + 0.001;
//        } else {
//          rotationPIDSettings.Ks = rotationPIDSettings.Ks + 0.01;
//        }
//      }
//    }
//    /*
//            if (controller1.getButtonStatus(ControllerWrapper.BUTTON_NAME.touchpad) == ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED) {
//                Pose2D target1 = new Pose2D(Field.getCoordinatesOfMiddleOfTile(new FieldTile('E', 4)), 90.0);
//                Pose2D target2 = new Pose2D(Field.getCoordinatesOfMiddleOfTile(new FieldTile('E', 5)), 90.0);
//                target1 = Pose2D.convertFieldCFAbsoluteToAllianceCFAbsolute(target1, AllianceInfo.ALLIANCE_COLOR.RED);
//                target2 = Pose2D.convertFieldCFAbsoluteToAllianceCFAbsolute(target2, AllianceInfo.ALLIANCE_COLOR.RED);
//
//                Waypoint waypoint1 = new Waypoint(target1);
//                Waypoint waypoint2 = new Waypoint(target2);
//                List<Waypoint> waypointList = Arrays.asList(waypoint1, waypoint2);// List<>();
//    //            waypointList.add(waypoint1);
//    //            waypointList.add(waypoint2);
//    */
//    /*            robot.qMultiDrivetrainRC("Test",
//                        MultiDrivetrainRC.COMMAND_TYPE.ABSOLUTE,
//                        waypointList,
//                        MultiDrivetrainRC.COMMAND_CONTENTS.TRANSLATION_AND_ROTATION,
//                        null, 10000,
//                        MultiDrivetrainRC.TARGET_TOLERANCE.FINE);
//
//            }
//    */
//  }
//
//  @Override
//  /** Create controllers for TeleOp. This function should be overridden for each season's game. */
//  protected void createControllersForTeleOp() {
//    super.createControllersForTeleOp();
//
//    // region ** Controller 1 **
//    // TODO: Add all the functions from processController1 below
//    //        controller1.addActiveButtonGroup(new
//    // ButtonToFunction(ControllerWrapper.BUTTON_NAME.square,
//    // ButtonWrapper.BUTTON_STATE.NEWLY_PRESSED, ControllerWrapper.OPTION_SHIFT, this::resetYaw));
//    // endregion ** Controller 1 **
//  }
//
//  /** Add items to telemetry display. */
//  @Override
//  protected void addTelemetryBody() {
//    super.addTelemetryBody();
//
//    telemetry.addLine("Currently selected movement type: " + movementType);
//    telemetry.addLine("Press touchpad to adjust either translation or rotation settings");
//    telemetry.addLine();
//
//    telemetry.addLine("DPAD for quantum translation movements. Bumpers for quantum rotations.");
//    telemetry.addLine();
//    telemetry.addLine("Use square to adjust Kp.");
//    telemetry.addLine("Use triangle to adjust Ki.");
//    telemetry.addLine("Use circle to adjust Kd.");
//    telemetry.addLine("Use cross to adjust Ks.");
//    telemetry.addLine("Hold share to decrease any of the above values.");
//
//    if ((xyPIDSettings != null) && (rotationPIDSettings != null)) {
//      telemetry.addLine(
//          "XY Ks="
//              + xyPIDSettings.Ks
//              + ", Kp="
//              + xyPIDSettings.Kp
//              + ", Ki="
//              + xyPIDSettings.Ki
//              + ", Kd="
//              + xyPIDSettings.Kd
//              + "\n");
//      telemetry.addLine(
//          "Rotation Ks="
//              + rotationPIDSettings.Ks
//              + ", Kp="
//              + rotationPIDSettings.Kp
//              + ", Ki="
//              + rotationPIDSettings.Ki
//              + ", Kd="
//              + rotationPIDSettings.Kd
//              + "\n");
//    }
//  }
//}
