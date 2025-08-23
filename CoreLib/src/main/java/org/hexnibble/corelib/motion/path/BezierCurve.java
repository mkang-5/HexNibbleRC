package org.hexnibble.corelib.motion.path;

import org.hexnibble.corelib.exception.InvalidArgumentException;
import org.hexnibble.corelib.misc.Field;
import org.hexnibble.corelib.misc.Msg;
import org.hexnibble.corelib.misc.Pose2D;
import org.hexnibble.corelib.misc.Vector2D;

import java.util.ArrayList;

public class BezierCurve extends CorePath {
   // This is enough for 8 points (including the start/end)
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

//   private final double curveLength;

   final int CURVE_LENGTH_INTEGRATION_STEPS = 100;
   int[][] curveLengthsLUT;

   private final double initialHeading;
   private final double finalHeading;

   private double lastTValue;

   /**
    * This creates a new Bezier curve using specified control points.
    *
    * @param controlPoints Points that define the curve
    */
   public BezierCurve(HEADING_INTERPOLATION headingInterpolation, Pose2D... controlPoints) {
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

      this.headingInterpolation = headingInterpolation;

      numControlPoints = controlPoints.length;
      curveDegree = numControlPoints - 1;

      for (Pose2D controlPose : controlPoints) {
         this.controlPoints.add(new Pose2D(controlPose));
      }

      curveLengthsLUT = createCurveLengthsLUT();

//      curveLength = getRemainingCurveLength(0.0);
      lastTValue = 0.0;
      initialHeading = this.controlPoints.get(0).heading;
      targetPose = this.controlPoints.get(curveDegree);

      if (headingInterpolation == HEADING_INTERPOLATION.FIXED) {
         finalHeading = initialHeading;
         targetPose.heading = initialHeading;
      }
      else {
         finalHeading = this.controlPoints.get(curveDegree).heading;
      }

      Msg.log(getClass().getSimpleName(), "Constructor", "Setting targetPose = " + targetPose);
   }

   @Override
   public Pose2D getPoseError(Pose2D currentPose) {
//      Msg.log(getClass().getSimpleName(), "getPoseError", "Looking for closest T value");
      Vector2D closestCurvePoint = new Vector2D(0.0, 0.0);
      double currentT = getClosestTValue(currentPose.getCoordsAsVector(), closestCurvePoint);
      Msg.log(getClass().getSimpleName(), "getPoseError", "Closest T value=" + currentT + ", Closest Point=" + closestCurvePoint);

      double pathLengthRemaining = Math.min(getRemainingCurveLength(currentT), 100.0);
//      Vector2D closestCurvePoint = getPoint(currentT);
      Vector2D tangentAtClosestPoint = getTangent(currentT);
//      double tangentXSign = tangentAtClosestPoint.x / Math.abs(tangentAtClosestPoint.x);
//      double tangentYSign = tangentAtClosestPoint.y / Math.abs(tangentAtClosestPoint.y);
//
//      if (isNaN(tangentXSign)) {
//         tangentXSign = 0.0;
//      }
//      if (isNaN(tangentYSign)) {
//         tangentYSign = 0.0;
//      }

      double slope = tangentAtClosestPoint.y / tangentAtClosestPoint.x;

      double theta = Math.atan(slope);

      if (tangentAtClosestPoint.y > 0.0) {
         if (slope < 0.0) {
            theta += Math.PI;
         }
      }
      else {
         if (slope > 0.0) {
            theta += Math.PI;
         }
      }

//      double pathX = pathLengthRemaining * Math.cos(theta) * tangentXSign;
//      double pathY = pathLengthRemaining * Math.sin(theta) * tangentYSign;

      double pathX = pathLengthRemaining * Math.cos(theta);
      double pathY = pathLengthRemaining * Math.sin(theta);

      Msg.log(getClass().getSimpleName(), "getPoseError", "currentX=" + currentPose.x + ", curvePtX=" + closestCurvePoint.x + ", pathRemainingX=" + pathX
         + "\ncurrentY=" + currentPose.y + ", curvePtY=" + closestCurvePoint.y + ", pathRemainingY=" + pathY + ", theta" + Math.toDegrees(theta));

      double xTerm, yTerm;
//      if (tangentAtClosestPoint.x < 0.0) {
         xTerm = (closestCurvePoint.x - currentPose.x) + pathX;
//      }
//      else {
//         xTerm = (closestCurvePoint.x - currentPose.x) + pathX;
//      }

//      if (tangentAtClosestPoint.y < 0.0) {
         yTerm = (closestCurvePoint.y - currentPose.y) + pathY;
//      }
//      else {
//         yTerm = (closestCurvePoint.y - currentPose.y) + pathY;
//      }

      Pose2D translationError = new Pose2D(new Vector2D(
            xTerm, yTerm),
            Field.addRadiansToIMUHeading(getHeading(currentT), -currentPose.heading)
      );

      Msg.log(getClass().getSimpleName(), "getPoseError", "xError=" + translationError.x + ", yError=" + translationError.y + ", hdgError(deg)=" + Math.toDegrees(translationError.heading));

      return translationError;
   }

   @Override
   public boolean isPathComplete(Pose2D currentPose) {
      Msg.log(getClass().getSimpleName(), "isPathComplete", "lastTValue=" + lastTValue);

      return lastTValue > 0.96;
   }

   /**
    * Return the remaining curve length at the specified t parameter.
    *
    * @return Remaining curve length.
    */
   protected double getRemainingCurveLength(double currentT) {
      int t = (int) (Math.clamp(currentT, 0.0, 1.0) * 100.0);

      return curveLengthsLUT[t][0];
   }

   /**
    * This approximates the length of the BezierCurve.
    * It's like a Riemann's sum, but for a parametric function's arc length.
    *
    * @return returns the approximated length of the BezierCurve.
    */
   private int[][] createCurveLengthsLUT() {
//      Msg.log(getClass().getSimpleName(), "createCurveLengthsLUT", "");
      int[][] curveLengthsLUT = new int[CURVE_LENGTH_INTEGRATION_STEPS + 1][1];
      curveLengthsLUT[CURVE_LENGTH_INTEGRATION_STEPS][0] = 0;
      Vector2D previousPoint = getPoint(1.0);
      Vector2D currentPoint;
      double curveLength = 0.0;

      // Start index at 1 since 0 is assigned to previousPoint
      for (int i = (CURVE_LENGTH_INTEGRATION_STEPS - 1); i > -1 ; i--) {
         currentPoint = getPoint(1.0 / CURVE_LENGTH_INTEGRATION_STEPS * (double) i);
         curveLength += Vector2D.getDistance(previousPoint, currentPoint);
         curveLengthsLUT[i][0] = (int) curveLength;
//         Msg.log(getClass().getSimpleName(), "createCurveLengthsLUT", "t=" + (1.0 / CURVE_LENGTH_INTEGRATION_STEPS * (double) i) + ", length=" + curveLength);
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

   protected double getHeading(double t) {
      switch (headingInterpolation) {
         case FIXED -> {
            return initialHeading;
         }
         case LINEAR -> {
            return Field.addRadiansToIMUHeading(initialHeading, (t * (finalHeading - initialHeading)));
         }
         default -> throw new IllegalStateException("Unexpected value: " + headingInterpolation);
      }
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
//      Msg.log(getClass().getSimpleName(), "getTangent", "Tangent vector at t=" + t + " is " + tangentVector);

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
    * @param currentPosition
    * @return parametric t value that is closest to the specified pose (clamped to between 0 and 1)
    */
//   @Override
   public double getClosestTValue(Vector2D currentPosition, Vector2D refClosestPointOnCurve) {
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
         double dist = Vector2D.getDistance(pointAtT, currentPosition);

         if (dist < closestDist) {
            closestDist = dist;
            closestT = t;
            refClosestPointOnCurve.setXY(pointAtT.x, pointAtT.y);
//            Msg.log(getClass().getSimpleName(), "getClosestTValue", "Setting closest point on curve to " + refClosestPointOnCurve);
         }
      }
      lastTValue = closestT;
      return closestT;
   }
}
