package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;

public class ArcPathV2 {

    /*
    This class was generated mostly by Gemini

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

   /**
    * Calculates points along a circular arc from pos1 to pos2,
    * with the arc's depth and direction determined by the parameters.
    * The R value of each returned Position is the tangent angle in degrees at that point.
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
   public static Position[] calculateArcPath(Position pos1, Position pos2, double depth, int direction, int numPositions) {
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
      double centerOffsetAngleRadians = chordAngleRadians - (direction * Math.PI / 2); // 90 degrees from chord direction based on 'direction'

      double centerX = chordMidpointX + distanceCenterToChordMidpoint * Math.cos(centerOffsetAngleRadians);
      double centerY = chordMidpointY + distanceCenterToChordMidpoint * Math.sin(centerOffsetAngleRadians);

      // 5. Calculate the Start and End Angles relative to the center
      double startAngleRadians = Math.atan2(pos1.Y - centerY, pos1.X - centerX);
      double endAngleRadians = Math.atan2(pos2.Y - centerY, pos2.X - centerX);

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

      // Ensure the magnitude of the sweep angle corresponds to the shorter arc segment
      // if the initial calculation yielded the longer one, unless it's a semicircle.
      // The distanceCenterToChordMidpoint calculation implies the center is on the side of the chord
      // that produces the arc defined by 'h', which corresponds to the shorter segment unless h is max (semicircle).
      // If the calculated sweep angle magnitude is greater than PI and the depth is not 1,
      // it means the center was calculated on the wrong side, or the angle calculation needs adjustment
      // to always get the shorter arc based on this center.
      // A simpler check: If the magnitude of the sweep angle is > PI and depth < 1,
      // we've likely calculated the longer arc. Take the shorter one (2*PI - |sweep|).
      if (Math.abs(sweepAngleRadians) > Math.PI + epsilon && depth < 1.0 - epsilon) {
         // If the calculated sweep is the longer arc segment, flip it.
         if (sweepAngleRadians > 0) { // Was calculated as large positive (CCW)
            sweepAngleRadians = -(2 * Math.PI - sweepAngleRadians); // Flip to shorter CW
         } else { // Was calculated as large negative (CW)
            sweepAngleRadians = (2 * Math.PI + sweepAngleRadians); // Flip to shorter CCW (sweepAngleRadians is negative)
         }

         // Now, adjust the sign based on the requested direction.
         if (direction == 1 && sweepAngleRadians < 0) { // Want CCW but got CW
            sweepAngleRadians = Math.abs(sweepAngleRadians);
         } else if (direction == -1 && sweepAngleRadians > 0) { // Want CW but got CCW
            sweepAngleRadians = -Math.abs(sweepAngleRadians);
         }
      }


      // 7. Calculate the Angle Increment
      double angleIncrementRadians = sweepAngleRadians / (numPositions - 1);

      // 8. Iterate and Calculate Points
      Position[] arcPoints = new Position[numPositions];
      for (int i = 0; i < numPositions; i++) {
         double currentAngleRadians = startAngleRadians + i * angleIncrementRadians;

         // Point Coordinates
         double pointX = centerX + radius * Math.cos(currentAngleRadians);
         double pointY = centerY + radius * Math.sin(currentAngleRadians);

         // Tangent Angle (90 degrees from the radius angle, direction depends on arc direction)
         // For counterclockwise, tangent is radius angle + PI/2.
         // For clockwise, tangent is radius angle - PI/2.
         double tangentAngleRadians = currentAngleRadians + (direction * Math.PI / 2);

         // Normalize tangent angle to [0, 360)
//         double tangentAngleDegrees = Math.toDegrees(tangentAngleRadians);
//         while (tangentAngleDegrees < 0) {
//            tangentAngleDegrees += 360;
//         }
//         while (tangentAngleDegrees >= 360) {
//            tangentAngleDegrees -= 360;
//         }
         double tangentAngleDegrees = Math.toDegrees(tangentAngleRadians);
         while (tangentAngleDegrees <= -180) {
            tangentAngleDegrees += 360;
         }
         while (tangentAngleDegrees > 180) {
            tangentAngleDegrees -= 360;
         }
//         while (A > 180) A -= 360;
//         while (A <= -180) A += 360;
//         return A;

         arcPoints[i] = new Position(pointX, pointY, tangentAngleDegrees);
      }

      return arcPoints;
   }

   // Example usage
   // Assuming the Position class is defined elsewhere or within this file
   // Example Position class definition (if not already present):
   public static class Position {
      public double X;
      public double Y;
      public double R; // Angle in degrees

      public Position(double X, double Y, double R) {
         this.X = X;
         this.Y = Y;
         this.R = R;
      }

      @Override
      public String toString() {
         return "Position(X=" + String.format("%.2f", X) + ", Y=" + String.format("%.2f", Y) + ", R=" + String.format("%.2f", R) + "°)";
      }
   }

   public static void main(String[] args) {
      // Example 1: Counterclockwise Arc from (0, 0) to (10, 0) with depth 0.5
      Position pos1_ex1 = new Position(0, 0, 0);
      Position pos2_ex1 = new Position(10, 0, 0);
      double depth_ex1 = 0.5;
      int direction_ex1 = 1; // Counterclockwise
      int numPositions_ex1 = 11;

      System.out.println("CCW Arc Points from (0,0) to (10,0) with depth 0.5:");
      Position[] arcPath_ex1 = calculateArcPath(pos1_ex1, pos2_ex1, depth_ex1, direction_ex1, numPositions_ex1);
      for (Position point : arcPath_ex1) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 2: Clockwise Arc from (0, 0) to (10, 0) with depth 0.5
      Position pos1_ex2 = new Position(0, 0, 0);
      Position pos2_ex2 = new Position(10, 0, 0);
      double depth_ex2 = 0.5;
      int direction_ex2 = -1; // Clockwise
      int numPositions_ex2 = 11;

      System.out.println("CW Arc Points from (0,0) to (10,0) with depth 0.5:");
      Position[] arcPath_ex2 = calculateArcPath(pos1_ex2, pos2_ex2, depth_ex2, direction_ex2, numPositions_ex2);
      for (Position point : arcPath_ex2) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 3: Counterclockwise Semicircle from (0, 0) to (0, 10) with depth 1.0
      Position pos1_ex3 = new Position(0, 0, 0);
      Position pos2_ex3 = new Position(0, 10, 0);
      double depth_ex3 = 1.0; // Semicircle
      int direction_ex3 = 1; // Counterclockwise
      int numPositions_ex3 = 11;

      System.out.println("CCW Semicircle Points from (0,0) to (0,10) with depth 1.0:");
      Position[] arcPath_ex3 = calculateArcPath(pos1_ex3, pos2_ex3, depth_ex3, direction_ex3, numPositions_ex3);
      for (Position point : arcPath_ex3) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 4: Clockwise Semicircle from (0, 0) to (0, 10) with depth 1.0
      Position pos1_ex4 = new Position(0, 0, 0);
      Position pos2_ex4 = new Position(0, 10, 0);
      double depth_ex4 = 1.0; // Semicircle
      int direction_ex4 = -1; // Clockwise
      int numPositions_ex4 = 11;

      System.out.println("CW Semicircle Points from (0,0) to (0,10) with depth 1.0:");
      Position[] arcPath_ex4 = calculateArcPath(pos1_ex4, pos2_ex4, depth_ex4, direction_ex4, numPositions_ex4);
      for (Position point : arcPath_ex4) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

      // Example 5: Counterclockwise Semicircle from (0, 0) to (10, 0) with depth 1.0
      Position pos1_ex5 = new Position(0, 0, 0);
      Position pos2_ex5 = new Position(10, 0, 0);
      double depth_ex5 = 1.0; // Semicircle
      int direction_ex5 = 1; // Counterclockwise
      int numPositions_ex5 = 11;

      System.out.println("CCW Semicircle Points from (0,0) to (10,0) with depth 1.0:");
      Position[] arcPath_ex5 = calculateArcPath(pos1_ex5, pos2_ex5, depth_ex5, direction_ex5, numPositions_ex5);
      for (Position point : arcPath_ex5) {
         System.out.println(point);
      }

      System.out.println("\n--------------------\n");

   }
}