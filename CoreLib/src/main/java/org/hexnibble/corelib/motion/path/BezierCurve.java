package org.hexnibble.corelib.motion.path;

import androidx.annotation.Nullable;

import org.hexnibble.corelib.exception.InvalidArgumentException;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;

import java.util.ArrayList;

public class BezierCurve extends CorePath {
   static final int[][] BINOMIAL_COEFFICIENT = {
         {1},
         {1, 1},
         {1, 2, 1},
         {1, 3, 3, 1},
         {1, 4, 6, 4, 1},
         {1, 5, 10, 10, 5, 1},
         {1, 6, 15, 20, 15, 6, 1},
         {1, 7, 21, 35, 35, 21, 7, 1}
   };

   private final int numControlPoints;       // Curve Order
   private final int curveDegree;            // Control Points - 1
   private ArrayList<Pose2D> controlPoints = new ArrayList<>();
//   private final int LENGTH_CALCULATION_STEPS = 50;
   private final double curveLength;

   final int CURVE_LENGTH_INTEGRATION_STEPS = 100;
   int[][] curveLengthsLUT;

   private double lastTValue;

   /**
    * This creates a new Bezier curve using specified control points.
    *
    * @param controlPoints Points that define the curve
    */
   public BezierCurve(Pose2D... controlPoints) {
      super(controlPoints[controlPoints.length - 1]);

      if ((controlPoints.length < 3) || (controlPoints.length > 8)) {
         String errorMsg = "Only Bezier curves with between 3 and 8 control points are supported--" + controlPoints.length + " were provided.";
         try {
            throw new InvalidArgumentException(errorMsg);
         }
         catch (InvalidArgumentException iae) {
            Msg.log(getClass().getSimpleName(), "Constructor", errorMsg);
            System.out.println(errorMsg);
            System.exit(0);
         }
      }

      numControlPoints = controlPoints.length;
      curveDegree = numControlPoints - 1;

      for (Pose2D controlPose : controlPoints) {
         this.controlPoints.add(new Pose2D(controlPose));
      }

      curveLengthsLUT = createCurveLengthsLUT();

      curveLength = getCurveLength();
      lastTValue = 0.0;

//      initialize();
   }

//   public void initialize() {
//      int n = controlPoints.size()-1;
//
//      length = approximateLength();
//      UNIT_TO_TIME = 1/length;
//      endTangent.setOrthogonalComponents(controlPoints.get(controlPoints.size()-1).getX()-controlPoints.get(controlPoints.size()-2).getX(), controlPoints.get(controlPoints.size()-1).getY()-controlPoints.get(controlPoints.size()-2).getY());
//      endTangent = MathFunctions.normalizeVector(endTangent);
//   }
//
//   @Override
//   public double getXError(Pose2D currentPose) {
//      // Need to decide what the target coordinate is, taking into account distance off the path
//      // and distance from the end of the curve
//      // Magnitude ~ distance to the path + distance remaining on the path -- should actually be
//      // the distance of a combined vector
//
//      Msg.log(getClass().getSimpleName(), "getXError", "Looking for closest T value");
//      Vector2D closestCurvePoint = new Vector2D(0.0, 0.0);
//      double currentT = getClosestTValue(currentPose, closestCurvePoint);
//      Msg.log(getClass().getSimpleName(), "getXError", "Closest T value=" + currentT + ", Closest Point=" + closestCurvePoint);
//
//      double pathRemaining = getRemainingCurveLength(currentT);
////      Vector2D closestCurvePoint = getPoint(currentT);
//      Vector2D tangentAtClosestPoint = getTangent(currentT);
//      double slope = tangentAtClosestPoint.y / tangentAtClosestPoint.x;
//      double theta = Math.atan(slope);
//      double x = pathRemaining * Math.cos(theta);
//
//      Msg.log(getClass().getSimpleName(), "getXError", "currentX=" + currentPose.x + ", curvePtX=" + closestCurvePoint.x + ", pathRemainingX=" + x);
//
//      Vector2D errorVector = new Vector2D(
//            (closestCurvePoint.x - currentPose.x) + x,
//            0.0
//      );
//
//      Msg.log(getClass().getSimpleName(), "getXError", "xError=" + errorVector.x);
//
//      return 0.0;
//   }

//   @Override
//   public double getYError(Pose2D currentPose) {
//      return 0.0;
//   }

   @Override
   public Pose2D getPoseError(Pose2D currentPose) {
//      Msg.log(getClass().getSimpleName(), "getPoseError", "Looking for closest T value");
      Vector2D closestCurvePoint = new Vector2D(0.0, 0.0);
      double currentT = getClosestTValue(currentPose, closestCurvePoint);
      Msg.log(getClass().getSimpleName(), "getPoseError", "Closest T value=" + currentT + ", Closest Point=" + closestCurvePoint);

      double pathLengthRemaining = getRemainingCurveLength(currentT);
//      Vector2D closestCurvePoint = getPoint(currentT);
      Vector2D tangentAtClosestPoint = getTangent(currentT);
      double tangentXSign = tangentAtClosestPoint.x / Math.abs(tangentAtClosestPoint.x);
      double tangentYSign = tangentAtClosestPoint.y / Math.abs(tangentAtClosestPoint.y);

      double slope = tangentAtClosestPoint.y / tangentAtClosestPoint.x;

      double theta = Math.atan(slope);

      if (slope < 0.0) {
         theta += Math.PI;
      }

      double pathX = pathLengthRemaining * Math.cos(theta);
      double pathY = pathLengthRemaining * Math.sin(theta);

      Msg.log(getClass().getSimpleName(), "getPoseError", "currentX=" + currentPose.x + ", curvePtX=" + closestCurvePoint.x + ", pathRemainingX=" + pathX
         + "\ncurrentY=" + currentPose.y + ", curvePtY=" + closestCurvePoint.y + ", pathRemainingY=" + pathY + ", theta" + Math.toDegrees(theta));

      double xTerm, yTerm;
//      if (tangentAtClosestPoint.x < 0.0) {
         xTerm = (closestCurvePoint.x - currentPose.x) + (tangentXSign * pathX);
//      }
//      else {
//         xTerm = (closestCurvePoint.x - currentPose.x) + pathX;
//      }

//      if (tangentAtClosestPoint.y < 0.0) {
         yTerm = (closestCurvePoint.y - currentPose.y) + (tangentYSign * pathY);
//      }
//      else {
//         yTerm = (closestCurvePoint.y - currentPose.y) + pathY;
//      }

      Pose2D translationError = new Pose2D(new Vector2D(
            xTerm,
            yTerm),
            0.0
      );

//      Msg.log(getClass().getSimpleName(), "getPoseError", "xError=" + translationError.x + ", yError=" + translationError.y + ", hdgError(deg)=");

      return translationError;
   }

   @Override
   public boolean isPathComplete(Pose2D currentPose) {
      Msg.log(getClass().getSimpleName(), "isPathComplete", "lastTValue=" + lastTValue);

      return lastTValue > 0.95;
   }

   /**
    * This approximates the length of the BezierCurve.
    * It's like a Riemann's sum, but for a parametric function's arc length.
    *
    * @return returns the approximated length of the BezierCurve.
    */
   protected double getCurveLength() {
      return getRemainingCurveLength(0.0);
   }

//   public double getRemainingCurveLength(double currentT) {
//      Msg.log(getClass().getSimpleName(), "getRemainingCurveLength", "currentT=" + currentT);
//      Vector2D previousPoint = getPoint(currentT);
//      Vector2D currentPoint;
//      double curveLength = 0.0;
//      double T_INTERVAL = 0.02;
//      final int numIntegrationSteps = (int) ((1.0 - currentT) / T_INTERVAL);
//
//      // Start index at 1 since 0 is assigned to previousPoint
//      for (int i = 1; i < (numIntegrationSteps + 1); i++) {
//         currentPoint = getPoint(currentT + ((double) i * T_INTERVAL));
//         curveLength += Vector2D.getDistance(previousPoint, currentPoint);
//         previousPoint = currentPoint;
//      }
//
//      Msg.log(getClass().getSimpleName(), "getRemainingCurveLength", "curveLength=" + curveLength);
//      return curveLength;
//   }

   protected double getRemainingCurveLength(double currentT) {
      int t = (int) (Math.clamp(currentT, 0.0, 1.0) * 100.0);
      int curveLength = curveLengthsLUT[t][0];

      Msg.log(getClass().getSimpleName(), "getRemainingCurveLength", "currentT=" + currentT + ", curveLength=" + curveLength);
      return curveLength;
   }

   private int[][] createCurveLengthsLUT() {
      Msg.log(getClass().getSimpleName(), "createCurveLengthsLUT", "");
      int[][] curveLengthsLUT = new int[CURVE_LENGTH_INTEGRATION_STEPS + 1][1];
      curveLengthsLUT[CURVE_LENGTH_INTEGRATION_STEPS][0] = 0;
      Vector2D previousPoint = getPoint(1.0);
      Vector2D currentPoint;
      double curveLength = 0.0;

      // Start index at 1 since 0 is assigned to previousPoint
//      for (int i = 1; i < (CURVE_LENGTH_INTEGRATION_STEPS + 1); i++) {
      for (int i = (CURVE_LENGTH_INTEGRATION_STEPS - 1); i > -1 ; i--) {
         currentPoint = getPoint(1.0 / CURVE_LENGTH_INTEGRATION_STEPS * (double) i);
         curveLength += Vector2D.getDistance(previousPoint, currentPoint);
         curveLengthsLUT[i][0] = (int) curveLength;
         Msg.log(getClass().getSimpleName(), "createCurveLengthsLUT", "t=" + (1.0 / CURVE_LENGTH_INTEGRATION_STEPS * (double) i) + ", length=" + curveLength);
         previousPoint = currentPoint;
      }

      return curveLengthsLUT;
   }

   /**
    * Retrieve the point on the Bezier curve corresponding to the specified parametric t value.
    *
    * @param t Parametric value on curve (from 0 - 1)
    * @return Point on curve.
    */
   public Vector2D getPoint(double t) {
      t = Math.clamp(t, 0, 1);
      double x = 0.0;
      double y = 0.0;

      // Each coordinate is the sum of a polynomial
      for (int i = 0; i < controlPoints.size(); i++) {
         double term = getCoefficient(curveDegree, i) * Math.pow(1 - t, curveDegree - i) * Math.pow(t, i);
         x += term * controlPoints.get(i).x;
         y += term * controlPoints.get(i).y;
      }

      Vector2D point = new Vector2D(x, y);

//      Msg.log(getClass().getSimpleName(), "getPoint", "Curve point at t=" + t + " is " + point);

      return point;
   }

   public Vector2D getTangent(double t) {
      t = Math.clamp(t, 0, 1);
      double x = 0.0;
      double y = 0.0;

      // Each coordinate is the sum of a polynomial
      for (int i = 0; i < (controlPoints.size() - 1); i++) {
         double term = getCoefficient(curveDegree - 1, i) * Math.pow(1 - t, curveDegree - i - 1) * Math.pow(t, i);
         x += term * (controlPoints.get(i + 1).x - controlPoints.get(i).x);
         y += term * (controlPoints.get(i + 1).y - controlPoints.get(i).y);
      }

      Vector2D tangentVector = new Vector2D(x, y);
      Msg.log(getClass().getSimpleName(), "getTangent", "Tangent vector at t=" + t + " is " + tangentVector);

      return tangentVector;
   }

   /**
    * Retrieve the binomial coefficient for the requested row and column
    * @param row 0-based row of the triangle
    * @param column 0-based column of the triangle
    * @return Binomial coefficent
    */
   private double getCoefficient(int row, int column) {
      return BINOMIAL_COEFFICIENT[row][column];
   }

   /**
    * Calculate the parametric t on an interpolated version of this line segment that is closest
    * to the specified pose. If the point falls outside the segment, t will be clamped
    * between 0 and 1 to keep it on the segment.
    * @param pose
    * @return parametric t value that is closest to the specified pose (clamped to between 0 and 1)
    */
//   @Override
   public double getClosestTValue(Pose2D pose, Vector2D refClosestPointOnCurve) {
      // First determine a rough t-value that is closest to the current pose
      // This is done by starting with the previous t-value and checking values around it as well
      // Store the t-value that gives the minimum distance to the current pose
      final double STEP_SIZE = 0.02;
      final int MAX_ONE_TAIL_VALUES = 10;

      int numLowerValues = (int) (lastTValue / STEP_SIZE) - 1;
      int numUpperValues = (int) ((1.0 - lastTValue) / STEP_SIZE) - 1;

      numLowerValues = Math.clamp(numLowerValues, 0, MAX_ONE_TAIL_VALUES);
      numUpperValues = Math.clamp(numUpperValues, 0, MAX_ONE_TAIL_VALUES);

      double[] tValuesToTest = new double[numLowerValues + numUpperValues + 1];

      tValuesToTest[0] = lastTValue;

      for (int i = 1; i < (numLowerValues + 1); i++) {
         tValuesToTest[i] = lastTValue - (i * STEP_SIZE);
      }

      for (int i = 1; i < (numUpperValues + 1); i++) {
         tValuesToTest[i + numLowerValues] = lastTValue + (i * STEP_SIZE);
      }

      double closestDist = 1e9;
      double closestT = 0.0;

      Msg.log(getClass().getSimpleName(), "getClosestTValue", "numLowerValues=" + numLowerValues + ", numUpperValues=" + numUpperValues + ", tValuesToTest=" + tValuesToTest.length);

      for (double t : tValuesToTest) {
         Vector2D pointAtT = getPoint(t);
         double dist = Vector2D.getDistance(pointAtT, pose.getCoordsAsVector());

         if (dist < closestDist) {
            closestDist = dist;
            closestT = t;
            refClosestPointOnCurve.setXY(pointAtT.x, pointAtT.y);
            Msg.log(getClass().getSimpleName(), "getClosestTValue", "Setting closest point on curve to " + refClosestPointOnCurve);
         }
      }
      lastTValue = closestT;
      return closestT;
   }
}
