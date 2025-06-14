package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.NavigationTarget;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
import org.firstinspires.ftc.teamcode.Tools.DataTypes.PositionTolerance;

import java.util.Arrays;

public class ArcPath {

    /*
    This class was initially generated mostly by Gemini
    (Specifically the "calculateArcPath" method, which has since been broken up, and the original tests.)

    My prompt:
      I have another change. Remember, I am looking for a function that creates points along a circular arc.
      I already have a class Position for robot poses with X coordinate, Y coordinate, R angle in degrees.
      Given two positions (pos1, pos2), the function will calculate positions along the arc from pos1 to pos2.
      Parameters to the function will be:
          pos1 – the initial position
          pos2 – the final position
          depth – the distance of the midpoint of the arc to the chord between pos1 and pos1, expressed as a fraction with a minimum of .1 and a maximum of 1
      direction – the direction of the arc: 1 for counterclockwise, -1 for clockwise.
          numPositions – the number of positions to calculate
      The arc will be circular. The centerpoint of the arc will be calculated using pos1, pos2, and depth.
      The function will return an array of Positions. Each position will include an R angle that is tangent to the arc.
      Please return the answer in multiple parts so that it will not be truncated.
   */

   /*
   Calculates points along a circular arc from pos1 to pos2,
   with the arc's depth and direction determining the arc.
   The R value of each returned Position is the tangent angle in degrees at that point.
   */

   // LK note:  For a right angle arc, the sagitta will be sqrt(2)-1 or 0.4142

   /**
    * Calculates positions ("breadcrumb trail") along a circular arc
    * using a start position, end position, depth (sagitta), and direction.
    * <p>The R value of each returned position is the tangent angle in degrees at that point.
    *
    * @param pos1           The initial Position of the arc. The R value is ignored.
    * @param pos2           The final Position of the arc. The R value is ignored.
    * @param depth          The distance of the arc's midpoint to the chord, as a fraction [0.1, 1].
    *                       A smaller depth means a shallower arc, a depth of 1 means the arc's midpoint
    *                       is such that the radius is minimal (semicircle).
    * @param direction      The direction of the arc. 1 for counterclockwise, -1 for clockwise.
    * @param numPositions   The number of positions to calculate along the arc.
    * @return An array of Position objects, or an empty array if calculation is not possible or invalid input.
    */
   public static Position[] calculateArcPathWithDepth(Position pos1, Position pos2, double depth, int direction, int numPositions) {
      // Input validation
      if (numPositions <= 1 || depth < 0.1 || depth > 1.0 || (direction != 1 && direction != -1)) {
         if (numPositions == 1) {
            // If only one point is requested, return the start position with a tangent
            // based on the initial chord direction. This is an approximation.
            double initialChordAngleRadians = Math.atan2(pos2.Y - pos1.Y, pos2.X - pos1.X);
            return new Position[]{new Position(pos1.X, pos1.Y, Math.toDegrees(initialChordAngleRadians))};
         } else {
            return new Position[0]; // Return empty array for invalid input
         }
      }

      // 1. Calculate the Midpoint of the Chord
      double chordMidpointX = (pos1.X + pos2.X) / 2.0;
      double chordMidpointY = (pos1.Y + pos2.Y) / 2.0;

      // 2. Calculate the Direction of the Chord and its length
      double deltaX_chord = pos2.X - pos1.X;
      double deltaY_chord = pos2.Y - pos1.Y;
      double chordLength = Math.sqrt(Math.pow(deltaX_chord, 2) + Math.pow(deltaY_chord, 2));

      if (chordLength < 1e-9) { // Handle case where pos1 and pos2 are very close or the same
         if (numPositions > 0) {
            // Return an array with just the single point (pos1), tangent could be pos1.R or based on context
            return new Position[]{new Position(pos1.X, pos1.Y, pos1.R)}; // Using pos1.R as a fallback
         }
         return new Position[0];
      }

      // Calculate the angle of the chord in radians
      double chordAngleRadians = Math.atan2(deltaY_chord, deltaX_chord);

      // 3. Calculate the Distance from Chord Midpoint to Arc Midpoint (sagitta, 'h')
      // Based on the interpretation that depth = 1 means a semicircle (radius = chordLength/2),
      // the sagitta h = depth * (chordLength / 2.0).
      double h = depth * (chordLength / 2.0);

      // Now, calculate the radius 'r' using the chord length 'c' and sagitta 'h': r = (c^2 + 4h^2) / 8h
      // Add a small epsilon to h in the denominator to avoid division by zero if depth is effectively 0
      double epsilon = 1e-9;
      double radius = (Math.pow(chordLength, 2) + 4 * Math.pow(h, 2)) / (8 * (h + epsilon));

      // Calculate the distance from the center to the chord midpoint
      double distanceCenterToChordMidpoint = radius - h;

      // 4. Calculate the Center Coordinates
      // The center is located from the chord midpoint, moving along the perpendicular bisector.
      // The direction from the chord midpoint to the center depends on the arc's direction.
      // For a counterclockwise arc from pos1 to pos2, the center is 90 degrees clockwise from the chord direction.
      // For a clockwise arc from pos1 to pos2, the center is 90 degrees counterclockwise from the chord direction.
      // This direction is opposite to the arc sweep direction.
      //double centerOffsetAngleRadians = chordAngleRadians - (direction * Math.PI / 2); // 90 degrees from chord direction based on 'direction'

      // LK: The generated code was exactly wrong/backward! For CCW arc, the center is 90 degrees CCW from the chord direction (and vice versa).
      double centerOffsetAngleRadians = chordAngleRadians + (direction * Math.PI / 2); // 90 degrees from chord direction based on 'direction'

//      double centerX = chordMidpointX + distanceCenterToChordMidpoint * Math.cos(centerOffsetAngleRadians);
//      double centerY = chordMidpointY + distanceCenterToChordMidpoint * Math.sin(centerOffsetAngleRadians);
      // LK debugging:  System.out.println("center x: " + centerX + " center y: " + centerY  );
      // LK: Converted to use Position for Center
      Position center = new Position();
      center.X = chordMidpointX + distanceCenterToChordMidpoint * Math.cos(centerOffsetAngleRadians);
      center.Y = chordMidpointY + distanceCenterToChordMidpoint * Math.sin(centerOffsetAngleRadians);

      return calculateArcPoints(pos1, pos2, center, direction, numPositions);
   }

   /**
    * Calculates positions ("breadcrumb trail") along a circular arc
    * using a start position, end position, center position, and direction.
    * <p>The R value of each returned position is the tangent angle in degrees at that point.
    *
    * @param pos1           The initial Position of the arc. The R value is ignored.
    * @param pos2           The final Position of the arc. The R value is ignored.
    * @param center         The center Position of the circular arc. The R value is ignored.
    * @param direction      The direction of the arc. 1 for counterclockwise, -1 for clockwise.
    * @param numPositions   The number of positions to calculate along the arc.
    * @return An array of Position objects.
    */
   public static Position[] calculateArcPoints(Position pos1, Position pos2, Position center, int direction, int numPositions) {
      // LK: Broke the original into two functions so we can generate an arc from three points and other variations
      // LK: todo: if this will remain public, need to check/sanitize parameters

      double epsilon = 1e-9;
      double depth = 0.5;  // todo: Fix this temporary hack
      double radius = Math.sqrt(Math.pow(pos1.X - center.X, 2) + Math.pow(pos1.Y - center.Y, 2));

      // 5. Calculate the Start and End Angles relative to the center
      double startAngleRadians = Math.atan2(pos1.Y - center.Y, pos1.X - center.X);
      double endAngleRadians = Math.atan2(pos2.Y - center.Y, pos2.X - center.X);

      // Normalize angles to [0, 2pi)
      if (startAngleRadians < 0) {
         startAngleRadians += 2 * Math.PI;
      }
      if (endAngleRadians < 0) {
         endAngleRadians += 2 * Math.PI;
      }

      // 6. Determine the Sweep Angle based on direction
      double sweepAngleRadians;
      if (direction == 1) { // Counterclockwise
         if (endAngleRadians >= startAngleRadians) {
            sweepAngleRadians = endAngleRadians - startAngleRadians;
         } else {
            sweepAngleRadians = (2 * Math.PI - startAngleRadians) + endAngleRadians;
         }
      } else { // Clockwise
         if (endAngleRadians <= startAngleRadians) {
            sweepAngleRadians = startAngleRadians - endAngleRadians;
         } else {
            sweepAngleRadians = startAngleRadians + (2 * Math.PI - endAngleRadians);
         }
         // Sweep angle for clockwise traversal should be negative for incremental calculation
         sweepAngleRadians *= -1;
      }

      // LK: I think this whole section is not necessary???  Leaving it commented out in case debugging shows that it's needed.
//      // Ensure the magnitude of the sweep angle corresponds to the shorter arc segment
//      // if the initial calculation yielded the longer one, unless it's a semicircle.
//      // The distanceCenterToChordMidpoint calculation implies the center is on the side of the chord
//      // that produces the arc defined by 'h', which corresponds to the shorter segment unless h is max (semicircle).
//      // If the calculated sweep angle magnitude is greater than PI and the depth is not 1,
//      // it means the center was calculated on the wrong side, or the angle calculation needs adjustment
//      // to always get the shorter arc based on this center.
//      // A simpler check: If the magnitude of the sweep angle is > PI and depth < 1,
//      // we've likely calculated the longer arc. Take the shorter one (2*PI - |sweep|).
//      if (Math.abs(sweepAngleRadians) > Math.PI + epsilon && depth < 1.0 - epsilon) {  //todo: clean this up
//         // If the calculated sweep is the longer arc segment, flip it.
//         if (sweepAngleRadians > 0) { // Was calculated as large positive (CCW)
//            sweepAngleRadians = -(2 * Math.PI - sweepAngleRadians); // Flip to shorter CW
//         } else { // Was calculated as large negative (CW)
//            sweepAngleRadians = (2 * Math.PI + sweepAngleRadians); // Flip to shorter CCW (sweepAngleRadians is negative)
//         }
//
//         // Now, adjust the sign based on the requested direction.
//         if (direction == 1 && sweepAngleRadians < 0) { // Want CCW but got CW
//            sweepAngleRadians = Math.abs(sweepAngleRadians);
//         } else if (direction == -1 && sweepAngleRadians > 0) { // Want CW but got CCW
//            sweepAngleRadians = -Math.abs(sweepAngleRadians);
//         }
//      }

      // 7. Calculate the Angle Increment
      double angleIncrementRadians = sweepAngleRadians / (numPositions - 1);

      // 8. Iterate and Calculate Points
      Position[] arcPoints = new Position[numPositions];
      for (int i = 0; i < numPositions; i++) {
         double currentAngleRadians = startAngleRadians + i * angleIncrementRadians;

         // Point Coordinates
         double pointX = center.X + radius * Math.cos(currentAngleRadians);
         double pointY = center.Y + radius * Math.sin(currentAngleRadians);

         // Tangent Angle (90 degrees from the radius angle, direction depends on arc direction)
         // For counterclockwise, tangent is radius angle + PI/2.
         // For clockwise, tangent is radius angle - PI/2.
         double tangentAngleRadians = currentAngleRadians + (direction * Math.PI / 2);
         double tangentAngleDegrees = Functions.normalizeAngle(Math.toDegrees(tangentAngleRadians));

         arcPoints[i] = new Position(pointX, pointY, tangentAngleDegrees);
      }

      return arcPoints;
   }

   public static void main(String[] args) {

      Position pos1_ex;
      Position pos2_ex;
      double depth_ex;
      int direction_ex;
      int numPositions_ex;
      Position[] arcPath_ex;

      // Example 1: Counterclockwise Arc from (0, 0) to (10, 0) with depth 0.5
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(10, 0, 0);
      depth_ex = 0.5;
      direction_ex = 1; // Counterclockwise
      numPositions_ex = 11;

      System.out.println("1. CCW Arc Points from (0,0) to (10,0) with depth 0.5:");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 1B: Counterclockwise Arc from (0, 0) to (0, 10) with depth 0.5
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(0, 10, 0);
      depth_ex = 0.5;
      direction_ex = 1; // Counterclockwise
      numPositions_ex = 11;

      System.out.println("1B. CCW Arc Points from (0,0) to (0,10) with depth 0.5:");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 2: Clockwise Arc from (0, 0) to (10, 0) with depth 0.5
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(10, 0, 0);
      depth_ex = 0.5;
      direction_ex = -1; // Clockwise
      numPositions_ex = 11;

      System.out.println("2. CW Arc Points from (0,0) to (10,0) with depth 0.5:");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 3: Counterclockwise Semicircle from (0, 0) to (0, 10) with depth 1.0
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(0, 10, 0);
      depth_ex = 1.0; // Semicircle
      direction_ex = 1; // Counterclockwise
      numPositions_ex = 11;

      System.out.println("3. CCW Semicircle Points from (0,0) to (0,10) with depth 1.0:");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 3B:
      Position[] arcPath_ex2;
      System.out.println("3B. Adjusted relative by angle 180");
      arcPath_ex2 = adjustArcPathHeadingRelative(arcPath_ex, 180);
      for (Position point : arcPath_ex2) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 3C:
      System.out.println("3C. Adjusted to angle 45");
      arcPath_ex2 = adjustArcPathHeadingConstant(arcPath_ex, 45);
      for (Position point : arcPath_ex2) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 3D:
      System.out.println("3D. Adjusted to aim constantly at (0,2)");
      arcPath_ex2 = adjustArcPathHeadingTarget(arcPath_ex, new Position(0,2));
      for (Position point : arcPath_ex2) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 3E:
      System.out.println("3E. Heading smoothly from -90 to -180 degrees");
      arcPath_ex2 = adjustArcPathHeadingStartEnd(arcPath_ex, -90, -180);
      for (Position point : arcPath_ex2) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 3F:
      System.out.println("3F. Heading immediately from -90 to -180 degrees");
      arcPath_ex2 = adjustArcPathHeadingEnd(arcPath_ex, -90, -180);
      for (Position point : arcPath_ex2) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 4: Clockwise Semicircle from (0, 0) to (0, 10) with depth 1.0
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(0, 10, 0);
      depth_ex = 1.0; // Semicircle
      direction_ex = -1; // Clockwise
      numPositions_ex = 11;

      System.out.println("4. CW Semicircle Points from (0,0) to (0,10) with depth 1.0:");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 5: Counterclockwise Semicircle from (0, 0) to (10, 0) with depth 1.0
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(10, 0, 0);
      depth_ex = 1.0; // Semicircle
      direction_ex = 1; // Counterclockwise
      numPositions_ex = 11;

      System.out.println("5. CCW Semicircle Points from (0,0) to (10,0) with depth 1.0:");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 6: Counterclockwise Semicircle from (0, 0) to (7, 7) with depth 1.0
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(7, 7, 0);
      depth_ex = 1.0; // Semicircle
      direction_ex = 1; // Counterclockwise
      numPositions_ex = 11;

//      System.out.println("6. CCW Semicircle Points from (0,0) to (7,7) with depth 1.0:");
      System.out.println("6. " + ((direction_ex==1) ? "CCW" : "CW") + " arc points from (" + pos1_ex.toString() +") to (" +
              pos2_ex.toString() +") with depth " + depth_ex +":");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 7: Clockwise Quarter-circle from (0, 0) to (7, -7) with depth "1" = 0.4142
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(7, -7, 0);
      depth_ex = 1; // For a quarter-circle
      //depth_ex = Math.sqrt(2) - 1; // for a right angle curve.
      direction_ex = -1; // clockwise
      numPositions_ex = 11;

      System.out.println("7. " + ((direction_ex==1) ? "CCW" : "CW") + " 90° arc points from (" + pos1_ex.toString() +") to (" +
              pos2_ex.toString() +") with depth " + depth_ex +":");
      arcPath_ex = calculateRightAngleArcPath(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 8: 3 point
      pos1_ex = new Position(-4, 0, 0);
      pos2_ex = new Position(0, -6, 0);
      Position posM = new Position (0, -2, 0);
      //depth_ex = 0.5; // Semicircle
      //depth_ex = Math.sqrt(2) - 1; // for a right angle curve.
      //direction_ex = -1; // clockwise
      numPositions_ex = 11;

      System.out.println("8. 3 arc points");
      arcPath_ex = calculateArcPathFrom3Points(pos1_ex, posM, pos2_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 9: Spline!
      pos1_ex = new Position(-8, 6, 0);
      pos2_ex = new Position(8, -6, 0);
      posM = null;
      depth_ex = 1;
      direction_ex = -1; // clockwise
      numPositions_ex = 6;

      System.out.println("9. Spline");
      arcPath_ex = calculateSplineApprox(pos1_ex, pos2_ex, posM, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      System.out.println("9B. Spline Reverse");
      arcPath_ex = calculateSplineApprox(pos1_ex, pos2_ex, posM, depth_ex, -direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      System.out.println("9C. Spline with Center");
      posM = new Position(0, 2, 0);
      arcPath_ex = calculateSplineApprox(pos1_ex, pos2_ex, posM, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 10: Shallower spline
      pos2_ex = new Position(-8, 4, 0);
      pos1_ex = new Position(8, -4, 0);
      posM = null;
      depth_ex = 0.5;
      direction_ex = -1; // clockwise
      numPositions_ex = 6;

      System.out.println("10. Shallow Spline");
      arcPath_ex = calculateSplineApprox(pos1_ex, pos2_ex, posM, depth_ex, direction_ex, numPositions_ex);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }
      System.out.println("\n--------------------\n");

      // Example 11: Combined arcs
      pos1_ex = new Position(0, 0, 0);
      pos2_ex = new Position(10, 0, 0);
      depth_ex = 0.5;
      direction_ex = 1; // Counterclockwise
      numPositions_ex = 6;

      System.out.println("11. Combined arcs");
      arcPath_ex = calculateArcPathWithDepth(pos1_ex, pos2_ex, depth_ex, direction_ex, numPositions_ex);
      arcPath_ex2 = calculateArcPathWithDepth(pos2_ex, pos1_ex, depth_ex, direction_ex, numPositions_ex);
      arcPath_ex = combinePaths(arcPath_ex, arcPath_ex2, true);
      for (Position point : arcPath_ex) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

   }

   // Stuff below here is not AI but LK

   /**
    * Calculates the center position of a circle from three positions defining an arc along the perimeter.
    * <p>The R value of the returned center position (which would otherwise have no meaning)
    * is used to return the direction, based on the middle position relative to the start and end,
    * where 1 is counterclockwise and -1 is clockwise.
    *
    * @param startPos       The start Position of the arc. The R value is ignored.
    * @param midPos         A Position along the arc between the start and end. The R value is ignored.
    * @param endPos         The end Position of the arc. The R value is ignored.
    * @return A Position for the center of the circle, with R value being the CCW/CW direction.
    */
   // Adapted from https://stackoverflow.com/questions/4103405/what-is-the-algorithm-for-finding-the-center-of-a-circle-from-three-points
   public static Position getCircleCenterFrom3Points(Position startPos, Position midPos, Position endPos) {
      // tested nominally OK 20250527
      if (startPos.isEqualXY(midPos) || startPos.isEqualXY(endPos) || endPos.isEqualXY(midPos)) {
         return null; // This won't work right if the three points aren't different
      }
      double ax = (startPos.X + midPos.X) / 2;
      double ay = (startPos.Y + midPos.Y) / 2;
      double ux = (startPos.Y - midPos.Y);
      double uy = (midPos.X - startPos.X);
      double bx = (midPos.X + endPos.X) / 2;
      double by = (midPos.Y + endPos.Y) / 2;
      double vx = (midPos.Y - endPos.Y);
      double vy = (endPos.X - midPos.X);
      double dx = ax - bx;
      double dy = ay - by;
      double vu = vx * uy - vy * ux;
      if (vu == 0)
         return null; // Points are collinear, so no unique solution
      double g = (dx * uy - dy * ux) / vu;
      Position center = new Position(bx + g * vx, by + g * vy);

      //figure out if this is CW or CCW
      double startAngle = Math.toDegrees(Math.atan2(startPos.Y - center.Y, startPos.X - center.X));
      double midAngle = Math.toDegrees(Math.atan2(midPos.Y - center.Y, midPos.X - center.X));
      double sweep = Functions.normalizeAngle(midAngle - startAngle);
      center.R = Math.signum(sweep);
      //the center.R has no meaning otherwise, so use it to return the direction

      return center;
   }

   /**
    * Calculates positions ("breadcrumb trail") along a circular arc
    * using three positions (startPos, midPos, endPos) to define the arc
    * (with center and direction being calculated from that).
    * <p>The R value of each returned position is the tangent angle in degrees at that point.
    *
    * @param startPos       The initial Position of the arc. The R value is ignored.
    * @param midPos         A Position along the arc between the start and end. The R value is ignored.
    * @param endPos         The final Position of the arc. The R value is ignored.
    * @param numPositions   The number of positions to calculate along the arc.
    * @return An array of Position objects.
    */
   public static Position[] calculateArcPathFrom3Points (Position startPos, Position midPos, Position endPos, int numPositions) {
      // tested nominally OK 20250527
      Position center = getCircleCenterFrom3Points(startPos, midPos, endPos);
      if (center == null) return null;
      return calculateArcPoints(startPos, endPos, center, (int)center.R, numPositions);
   }

   /**
    * Calculates positions ("breadcrumb trail") along a circular arc
    * using a start position, end position, depth (sagitta), and direction.
    * The depth is scaled (by sqrt(2)-1) so that a value of 1 is suitable for
    * a right angle (quarter circle) path.
    * <p>The R value of each returned position is the tangent angle in degrees at that point.
    *
    * @param pos1           The initial Position of the arc. The R value is ignored.
    * @param pos2           The final Position of the arc. The R value is ignored.
    * @param depth          The distance of the arc's midpoint to the chord, as a fraction [0.1, 1].
    *                       A smaller depth means a shallower arc.
    *                       A depth of 1 is scaled for a 90 degree path (quarter circle).
    * @param direction      The direction of the arc. 1 for counterclockwise, -1 for clockwise.
    * @param numPositions   The number of positions to calculate along the arc.
    * @return An array of Position objects.
    */
   public static Position[] calculateRightAngleArcPath(Position pos1, Position pos2, double depth, int direction, int numPositions) {
      // tested nominally OK 20250527
      //For a right angle arc (versus semicircle type_, the sagitta will be sqrt(2)-1 or 0.4142
      return calculateArcPathWithDepth(pos1, pos2, depth * (Math.sqrt(2) - 1), direction, numPositions);
   }

   public static Position getCirclePoint(Position center, double radius, double angle) {
      return new Position(
         center.X + radius * Math.cos(Math.toRadians(angle)),
         center.Y + radius * Math.sin(Math.toRadians(angle)),
         Functions.normalizeAngle(angle + Math.signum(angle) * 90)
      );
   }

   // Things previously wanted and now added:
   // - A relative adjustment, e.g., +180 for the robot going backwards, +90 fot the robot facing the center
   // - Hold a constant angle (absolute)
   // - Hold a constant reference angle to some point (like the scoring target)
   // - Turn from pos1.R to pos2.R divided by the number of points (need to figure out or specify direction)
   // - Turn from pos1.R to pos2.R automatically like a linear navigation point
   // - A spline approximation method

   public static Position[] adjustArcPathHeadingRelative(Position[] path, double adjAngle) {
      // tested nominally OK 20250527
      if (path==null) return null;
      Position[] newPath = clonePath(path);
      for (Position p : newPath) {
         p.R = Functions.normalizeAngle(p.R + adjAngle);
      }
      return newPath;
   }

   public static Position[] adjustArcPathHeadingConstant(Position[] path, double constAngle) {
      // tested nominally OK 20250527
      if (path==null) return null;
      Position[] newPath = clonePath(path);
      double angle = Functions.normalizeAngle(constAngle);
      for (Position p : newPath) {
         p.R = angle;
      }
      return newPath;
   }

   public static Position[] adjustArcPathHeadingTarget(Position[] path, Position targetPos) {
      // tested nominally OK 20250527
      if (path==null) return null;
      Position[] newPath = clonePath(path);
      for (Position p : newPath) {
         double x = targetPos.X - p.X;
         double y = targetPos.Y - p.Y;
         p.R = Math.toDegrees(Math.atan2(y,x));
      }
      return newPath;
   }

   public static Position[] adjustArcPathHeadingStartEnd(Position[] path, double startHeading, double endHeading) {
      // tested nominally OK 20250527
      if (path==null) return null;
      Position[] newPath = clonePath(path);
      int numPos = newPath.length;
      //// determine direction consistent with autodrive; move to shorter error
      //int direction = (int)Math.signum(Functions.normalizeAngle(endHeading-startHeading));
      double interval = Functions.normalizeAngle(endHeading-startHeading) / (numPos - 1);
      for (int i = 0; i < newPath.length; i++) {
         newPath[i].R = Functions.normalizeAngle(interval * i + startHeading);
      }
      return newPath;
   }

   public static Position[] adjustArcPathHeadingEnd(Position[] path, double startHeading, double endHeading) {
      // tested nominally OK 20250527
      // this will only be useful with a positionTolerance that effectively ignores the heading
      if (path==null) return null;
      Position[] newPath = clonePath(path);
      for (Position p : newPath) {
         p.R = endHeading;
      }
      newPath[0].R = startHeading;
      return newPath;
   }

   public static Position[] calculateSplineApprox (Position pos1, Position pos2, Position center,
                                                   double depth, int direction, int numPositions) {
      // tested nominally OK 20250527
      // todo: replace this with something that can calculate a true spline?
      // calculates a pair of arc to simulate a spline; one weakness is that the midpoint won't be tangent
      // for simplicity, numPositions will be used for both arc segments, so the array will have length (numPositions * 2) - 1 (center is de-duplicated_
      // supply null for center if you want the midpoint calculated
      Position posCenter;
      if (center != null) {
         posCenter = center.clone();
      }
      else {
         posCenter = new Position ((pos1.X + pos2.X) / 2.0, (pos1.Y + pos2.Y) / 2.0);
      }
      Position[] firstArc  = calculateRightAngleArcPath(pos1, posCenter, depth,  direction, numPositions);
      Position[] secondArc = calculateRightAngleArcPath(posCenter, pos2, depth, -direction, numPositions);
//      // todo: 20250612 change this to use combinePaths
//      // Let's average out the middle position's angle, for which there are two equal positions with different headings...
//      secondArc[0].R = Functions.normalizeAngle((firstArc[firstArc.length - 1].R + secondArc[0].R)/2.0);
//      // And then drop the last position from the first arc (note the " - 1" for firstArc.length when building the new array
//      Position[] spline = Arrays.copyOf(firstArc, firstArc.length - 1 + secondArc.length);
//      System.arraycopy(secondArc, 0, spline, firstArc.length - 1, secondArc.length);
       return combinePaths(firstArc, secondArc, true);
   }

   public static Position[] clonePath (Position[] path) {
      // tested nominally OK 20250527
      if (path==null) return null;
      Position[] newPath = new Position[path.length];
      for (int i = 0; i < path.length; i++) {
         newPath[i] = path[i].clone();       // Using clone method
         //newPath[i] = new Position(path[i]); // Using copy constructor
      }
      return newPath;
   }

   public static Position[] combinePaths (Position[] path1, Position[] path2, boolean deleteMidpoint) {
      Position[] newPath = Arrays.copyOf(path1, path1.length + path2.length - (deleteMidpoint ? 1 : 0));
      System.arraycopy(path2, 0, newPath, path1.length - (deleteMidpoint ? 1 : 0), path2.length);
//      // todo: 20250612 fix this... need to account for the shorter arc, not simply /2. oops.
//      // possible references...
//      // https://stackoverflow.com/questions/1158909/average-of-two-angles-with-wrap-around
//      // https://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data
//      if (deleteMidpoint) {
//         newPath[path1.length -1].R = Functions.normalizeAngle((path1[path1.length - 1].R + path2[0].R)/2.0);
//      }
      if (deleteMidpoint) {
         newPath[path1.length - 1].R = Functions.averageAngle(path1[path1.length - 1].R, path2[0].R);
      }
      return newPath;
   }

   public static NavigationTarget[] buildNavTargetArray (Position[] path, PositionTolerance startTolerance, PositionTolerance midTolerance,
                                                         PositionTolerance endTolerance, double maxSpeed, long timeLimit,
                                                         boolean noSlowAtEnd ) {
      NavigationTarget[] navTargetArray = new NavigationTarget[path.length];
      for (int i = 0; i < path.length; i++) {
         navTargetArray[i] = new NavigationTarget(new Position(path[i]), midTolerance, maxSpeed, timeLimit, true);
      }
      navTargetArray[0].tolerance = startTolerance;
      navTargetArray[path.length - 1].tolerance = endTolerance;
      navTargetArray[path.length - 1].noSlow = noSlowAtEnd;
      return navTargetArray;
   }
   public static NavigationTarget[] buildNavTargetArray (Position[] path, PositionTolerance midTolerance,
                                                         PositionTolerance endTolerance, double maxSpeed, long timeLimit,
                                                         boolean noSlowAtEnd ) {
      return buildNavTargetArray(path, midTolerance, midTolerance, endTolerance, maxSpeed, timeLimit, noSlowAtEnd);
   }
   public static NavigationTarget[] buildNavTargetArray (Position[] path,
                                                         PositionTolerance endTolerance, double maxSpeed, long timeLimit,
                                                         boolean noSlowAtEnd ) {
      //todo: make a way to set the default transition tolerance
      PositionTolerance transitionTolerance = new PositionTolerance(4.0,90.0,0);
      return buildNavTargetArray(path, transitionTolerance, transitionTolerance, endTolerance, maxSpeed, timeLimit, noSlowAtEnd);
   }
}