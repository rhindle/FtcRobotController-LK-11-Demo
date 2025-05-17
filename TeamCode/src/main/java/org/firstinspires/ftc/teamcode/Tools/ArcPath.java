//package org.firstinspires.ftc.teamcode.Tools;
//
//import org.firstinspires.ftc.teamcode.RobotParts.Common.AutoDrive;
//import org.firstinspires.ftc.teamcode.Tools.DataTypes.Position;
//
//public class ArcPath {
//
//    /* Most of this was prepared by Gemini
//
//    == Here was the prompt ==
//
//    Let's redo this a little differently. I already have a class Position for robot poses with X coordinate,
//    Y coordinate, R angle in degrees. Given two positions (pos1, pos2), the function will calculate positions
//    along the arc from pos1 to pos2. Parameters to the function will be:
//      pos1 – the initial position
//      pos2 – the final position
//      depth – the distance of the midpoint of the arc to the chord between pos1 and pos1, expressed as a fraction with a minimum of .1 and a maximum of 1
//      numPositions – the number of positions to calculate
//    The arc will be circular. The centerpoint of the arc will be calculated using pos1, pos2, and depth.
//    The function will return an array of Positions. Each position will include an R angle that is tangent to the arc.
//
//    == Here was the response, plus code =
//
//    Okay, this is a more specific and interesting challenge! We'll be generating a circular arc defined by two endpoints (pos1, pos2) and a "depth" parameter, rather than explicitly providing the center. The depth determines how "deep" or curved the arc is. Here's how we can approach this:
//        Calculate the Midpoint of the Chord: Find the midpoint of the straight line segment (the chord) connecting pos1 and pos2.
//        Calculate the Perpendicular Bisector: The center of the circular arc must lie on the perpendicular bisector of the chord connecting pos1 and pos2. The direction of this bisector is 90 degrees rotated from the direction of the chord.
//        Calculate the Distance to the Center: The depth parameter gives us the distance from the chord midpoint along the perpendicular bisector to a point on the arc (the arc's midpoint). This information can be used to calculate the radius of the circle and, subsequently, the location of the center.
//        Calculate the Center Coordinates: Move from the chord midpoint along the perpendicular bisector direction by a calculated distance to find the center of the circle.
//        Calculate Points along the Arc: Once the center and radius are known, we can use the method from our previous discussion to calculate points along the arc and their tangent angles. Let's implement this in Java:
//
//    == Whoops! ==
//
//    I forgot to ask for a direction parameter when creating the prompt, so added it myself while reviewing the code.
//    Also a parameter stayTangent to determine whether to generate orientations tangent to the arc.
//    //todo: This needs more thought. Follow curve with front of robot? Back of robot? pos1.R to pos2.R?  ????
//
//     */
//
//    /**
//     * Calculates points along a circular arc from pos1 to pos2,
//     * with the arc's depth determined by the 'depth' parameter.
//     * The R value of each returned Position is the tangent angle in degrees at that point.
//     * Assumes counterclockwise traversal from pos1 to pos2.
//     *
//     * @param pos1           The initial Position of the arc. The R value is ignored.
//     * @param pos2           The final Position of the arc. The R value is ignored.
//     * @param depth          The distance of the arc's midpoint to the chord, as a fraction [0.1, 1].
//     *                       A smaller depth means a shallower arc, a depth of 1 means the arc's midpoint
//     *                       is such that the radius is minimal (semicircle).
//     * @param direction      The direction of the arc. 1 for counterclockwise, -1 for clockwise.
//     * @param numPositions   The number of positions to calculate along the arc.
//     * @param stayTangent    Whether to generate orientations tangent to the arc.
//     * @return An array of Position objects, or an empty array if calculation is not possible or invalid input.
//     */
//    public static Position[] calculateArcPath(Position pos1, Position pos2, double depth, int direction, int numPositions, boolean stayTangent) {
//        // Input validation
//        if (numPositions <= 1 || depth < 0.1 || depth > 1.0 || (direction != 1 && direction != -1)) {
//            if (numPositions == 1) {
//                // If only one point is requested, return the start position with a tangent
//                // based on the initial chord direction. This is an approximation.
//                double initialChordAngleRadians = Math.atan2(pos2.Y - pos1.Y, pos2.X - pos1.X);
//                return new Position[]{new Position(pos1.X, pos1.Y, Math.toDegrees(initialChordAngleRadians))};
//            } else {
//                return new Position[0]; // Return empty array for invalid input
//            }
//        }
//
//        // 1. Calculate the Midpoint of the Chord
////        Position chordMidpoint = new Position();
////        chordMidpoint.X = (pos1.X + pos2.X) / 2.0;
////        chordMidpoint.Y = (pos1.Y + pos2.Y) / 2.0;
//        double chordMidpointX = (pos1.X + pos2.X) / 2.0;
//        double chordMidpointY = (pos1.Y + pos2.Y) / 2.0;
//
//        // 2. Calculate the Direction of the Chord and its length
//        double deltaX_chord = pos2.X - pos1.X;
//        double deltaY_chord = pos2.Y - pos1.Y;
//        double chordLength = Math.sqrt(Math.pow(deltaX_chord, 2) + Math.pow(deltaY_chord, 2));
//
//        if (chordLength < 1e-9) { // Handle case where pos1 and pos2 are very close or the same
//            // Return an array with just the single point (pos1), tangent could be pos1.R or based on context
//            return new Position[]{new Position(pos1.X, pos1.Y, pos1.R)}; // Using pos1.R as a fallback
//        }
//
//        // Calculate the angle of the chord in radians
//        double chordAngleRadians = Math.atan2(deltaY_chord, deltaX_chord);
//
//        // 3. Calculate the Distance from Chord Midpoint to Arc Midpoint (sagitta, 'h')
//        // Based on the interpretation that depth = 1 means a semicircle (radius = chordLength/2),
//        // the sagitta h = depth * (chordLength / 2.0).
//        double h = depth * (chordLength / 2.0);
//
//        // Now, calculate the radius 'r' using the chord length 'c' and sagitta 'h': r = (c^2 + 4h^2) / 8h
//        // Add a small epsilon to h in the denominator to avoid division by zero if depth is effectively 0
//        double epsilon = 1e-9;
//        double radius = (Math.pow(chordLength, 2) + 4 * Math.pow(h, 2)) / (8 * (h + epsilon));
//
//        // Calculate the distance from the center to the chord midpoint
//        double distanceCenterToChordMidpoint = radius - h;
//
//        // 4. Calculate the Center Coordinates
//        // The center is located from the chord midpoint, moving along the perpendicular bisector.
//        // The direction from the chord midpoint to the center depends on whether the arc is "above" or "below" the chord.
//        // For a counterclockwise arc from pos1 to pos2, the center will be on the side of the chord
//        // such that the sweep angle from the center is less than PI for a segment less than a semicircle,
//        // and greater than PI for a segment more than a semicircle.
//        // The depth parameter implies the arc bends in a specific direction relative to the chord.
//        // Let's assume the standard interpretation where increasing depth makes the arc "deeper" on one side of the chord.
//        // The perpendicular bisector direction is chordAngleRadians + PI/2.
//        // The center is along this bisector, at a distance of `distanceCenterToChordMidpoint` from the chord midpoint.
//        // The direction from the chord midpoint to the center is perpendicular to the chord.
//        // If the arc is counterclockwise from pos1 to pos2, the center is generally 90 degrees clockwise from the chord direction.
////        double centerAngleRadians = chordAngleRadians - Math.PI / 2; // 90 degrees clockwise from chord
//        double centerAngleRadians = chordAngleRadians - (direction * Math.PI / 2); // 90 degrees from chord direction CCW or CW [LK]
//
//        double centerX = chordMidpointX + distanceCenterToChordMidpoint * Math.cos(centerAngleRadians);
//        double centerY = chordMidpointY + distanceCenterToChordMidpoint * Math.sin(centerAngleRadians);
//
//        // 5. Calculate the Start and End Angles relative to the center
//        double startAngleRadians = Math.atan2(pos1.Y - centerY, pos1.X - centerX);
//        double endAngleRadians = Math.atan2(pos2.Y - centerY, pos2.X - centerX);
//
//        // Normalize angles to [0, 2pi)
//        if (startAngleRadians < 0) {
//            startAngleRadians += 2 * Math.PI;
//        }
//        if (endAngleRadians < 0) {
//            endAngleRadians += 2 * Math.PI;
//        }
//
//        // 6. Determine the Sweep Angle
//        // Assuming counterclockwise traversal from pos1 to pos2.
//        // If the calculated end angle is less than the start angle, it means the arc wraps around 360 degrees.
//        double sweepAngleRadians;
//        if (endAngleRadians >= startAngleRadians) {
//            sweepAngleRadians = endAngleRadians - startAngleRadians;
//        } else {
//            sweepAngleRadians = (2 * Math.PI - startAngleRadians) + endAngleRadians;
//        }
//
//        // We need to be careful about the direction of the arc implied by the depth and the calculated center.
//        // The calculated center assumes a counterclockwise arc from pos1 to pos2.
//        // If the 'depth' is such that the arc should bend the other way (implying a clockwise arc),
//        // our calculated sweep angle will be the larger reflex angle (> PI).
//        // If the intended arc is counterclockwise, the sweep angle should be the smaller angle (< PI)
//        // unless it's a semicircle or more.
//        // The sign of the cross product (pos2-pos1) x (center-pos1) tells us which side of the chord the center is on.
//        // For a counterclockwise arc from pos1 to pos2, the center should be on the "left" side of the vector pos1->pos2.
//        // Cross product: (pos2.X - pos1.X) * (centerY - pos1.Y) - (pos2.Y - pos1.Y) * (centerX - pos1.X)
//        double crossProduct = (pos2.X - pos1.X) * (centerY - pos1.Y) - (pos2.Y - pos1.Y) * (centerX - pos1.X);
//
//        // If the cross product is negative (center is on the right), and we intended a counterclockwise arc,
//        // it means the calculated center is for the "other" arc segment.
//        // We should use the reflex angle (360 - sweep) in this case, or perhaps recalculate the center
//        // on the other side of the chord if the depth definition implies direction.
//        // Given the depth parameter defining the sagitta, the center location is uniquely determined on one side of the chord.
//        // The sign of 'h' (sagitta) is implicitly positive in our calculation, meaning the arc midpoint is away from the chord midpoint
//        // in the direction of the perpendicular bisector.
//        // If the sweep angle calculated is the larger angle (reflex angle), and we expect a counterclockwise arc segment that is less than a full circle,
//        // it means we should take the smaller angle (360 - sweep).
//        // A semicircle (depth=1) has a sweep of PI radians (180 degrees). If sweep > PI and depth < 1, something might be off.
//        // A more robust way is to check if the sweep angle corresponds to the side implied by the center calculation.
//        // If crossProduct is negative, and sweepAngleRadians is less than PI, we likely calculated the center for the wrong arc.
//        // If crossProduct is positive, and sweepAngleRadians is greater than PI (and not a full circle), we likely calculated the center for the wrong arc.
//
//        // Let's simplify and assume the depth always defines the arc segment that is less than or equal to a semicircle (sweep <= PI),
//        // unless pos1 and pos2 are the same (full circle case handled earlier).
//        // If the calculated sweep is > PI, take 2*PI - sweep, and potentially adjust the center side.
//        if (sweepAngleRadians > Math.PI * 1.001 && chordLength > 1e-9) { // Allow small tolerance
//            sweepAngleRadians = 2 * Math.PI - sweepAngleRadians;
//            // If we flipped the sweep angle, the center should be on the opposite side.
//            // This would require recalculating the center based on the *other* perpendicular bisector direction (chordAngleRadians + PI/2 + PI).
//            // However, based on the sagitta 'h' and radius 'r' formula, the distance from center to chord midpoint is fixed.
//            // The direction of this distance is what flips.
//            centerAngleRadians = chordAngleRadians + Math.PI / 2; // 90 degrees counterclockwise from chord
//            centerX = chordMidpointX + distanceCenterToChordMidpoint * Math.cos(centerAngleRadians);
//            centerY = chordMidpointY + distanceCenterToChordMidpoint * Math.sin(centerAngleRadians);
//
//            // Recalculate start and end angles with the new center
//            startAngleRadians = Math.atan2(pos1.Y - centerY, pos1.X - centerX);
//            endAngleRadians = Math.atan2(pos2.Y - centerY, pos2.X - centerX);
//            if (startAngleRadians < 0) startAngleRadians += 2 * Math.PI;
//            if (endAngleRadians < 0) endAngleRadians += 2 * Math.PI;
//
//            // Recalculate sweep angle (should now be <= PI)
//            if (endAngleRadians >= startAngleRadians) {
//                sweepAngleRadians = endAngleRadians - startAngleRadians;
//            } else {
//                sweepAngleRadians = (2 * Math.PI - startAngleRadians) + endAngleRadians;
//            }
//
//        }
//
//
//        // 7. Calculate the Angle Increment
//        double angleIncrementRadians = sweepAngleRadians / (numPositions - 1);
//
//        // 8. Iterate and Calculate Points
//        Position[] arcPoints = new Position[numPositions];
//        for (int i = 0; i < numPositions; i++) {
//            double currentAngleRadians = startAngleRadians + i * angleIncrementRadians;
//
//            // Point Coordinates
//            double pointX = centerX + radius * Math.cos(currentAngleRadians);
//            double pointY = centerY + radius * Math.sin(currentAngleRadians);
//
//            // Tangent Angle (90 degrees counterclockwise from the radius angle)
//            // For a counterclockwise arc, the tangent angle at a point on the arc is
//            // the angle of the radius to that point plus 90 degrees (PI/2 radians).
//            double tangentAngleRadians = currentAngleRadians + Math.PI / 2;
//
//            // Normalize tangent angle to [0, 360)
//            double tangentAngleDegrees = Math.toDegrees(tangentAngleRadians);
//            if (tangentAngleDegrees < 0) {
//                tangentAngleDegrees += 360;
//            } else if (tangentAngleDegrees >= 360) {
//                tangentAngleDegrees -= 360;
//            }
//
//            arcPoints[i] = new Position(pointX, pointY, tangentAngleDegrees);
//        }
//
//        return arcPoints;
//    }
//
//    // Example usage
//    public static void main(String[] args) {
//        AutoDrive autoDrive = new AutoDrive();
//
//        // Example 1: Arc from (0, 0) to (10, 0) with depth 0.5
//        // This should be an arc above the x-axis.
//        Position pos1_ex1 = new Position(0, 0, 0);
//        Position pos2_ex1 = new Position(10, 0, 0);
//        double depth_ex1 = 0.5; // Halfway to a semicircle
//        int numPositions_ex1 = 10;
//
//        System.out.println("Arc Points from (0,0) to (10,0) with depth 0.5:");
//        Position[] arcPath_ex1 = ArcPath.calculateArcPath(pos1_ex1, pos2_ex1, depth_ex1, numPositions_ex1);
//        for (Position point : arcPath_ex1) {
//            System.out.println(point);
//        }
//
//        System.out.println("\n--------------------\n");
//
//        // Example 2: Arc from (0, 0) to (0, 10) with depth 1.0 (semicircle)
//        // This should be a semicircle to the left of the y-axis.
//        Position pos1_ex2 = new Position(0, 0, 0);
//        Position pos2_ex2 = new Position(0, 10, 0);
//        double depth_ex2 = 1.0; // Semicircle
//        int numPositions_ex2 = 10;
//
//        System.out.println("Arc Points from (0,0) to (0,10) with depth 1.0 (semicircle):");
//        Position[] arcPath_ex2 = ArcPath.calculateArcPath(pos1_ex2, pos2_ex2, depth_ex2, numPositions_ex2);
//        for (Position point : arcPath_ex2) {
//            System.out.println(point);
//        }
//
//        System.out.println("\n--------------------\n");
//
//        // Example 3: Arc from (0, 0) to (10, 10) with depth 0.8
//        Position pos1_ex3 = new Position(0, 0, 0);
//        Position pos2_ex3 = new Position(10, 10, 0);
//        double depth_ex3 = 0.8;
//        int numPositions_ex3 = 10;
//
//        System.out.println("Arc Points from (0,0) to (10,10) with depth 0.8:");
//        Position[] arcPath_ex3 = ArcPath.calculateArcPath(pos1_ex3, pos2_ex3, depth_ex3, numPositions_ex3);
//        for (Position point : arcPath_ex3) {
//            System.out.println(point);
//        }
//    }
//
//
//}
