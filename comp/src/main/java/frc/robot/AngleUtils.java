package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class AngleUtils {
  /**
   * Calculates the smallest difference between two angles.
   * The result is in the range [0, 180] degrees.
   *
   * @param angle1
   *          The first angle.
   * @param angle2
   *          The second angle.
   * @return The smallest difference between the two angles.
   */
  public static Rotation2d smallestAngleDifference(Rotation2d angle1, Rotation2d angle2) {
    double difference = Math.abs(angle1.getDegrees() - angle2.getDegrees()) % 360;
    return Rotation2d.fromDegrees(difference > 180 ? 360 - difference : difference);
  }

  /**
   * Checks if two angles are within a specified tolerance.
   *
   * @param angle1
   *          The first angle.
   * @param angle2
   *          The second angle.
   * @param tolerance
   *          The tolerance.
   * @return True if the angles are within the tolerance; false otherwise.
   */
  public static boolean areAnglesClose(Rotation2d angle1, Rotation2d angle2, Rotation2d tolerance) {
    return smallestAngleDifference(angle1, angle2).getDegrees() <= tolerance.getDegrees();
  }

  /**
   * Calculates the signed smallest difference between two angles.
   * The result is in the range [-180, 180) degrees.
   *
   * @param angle1
   *          The first angle.
   * @param angle2
   *          The second angle.
   * @return The signed smallest difference between the two angles.
   */
  public static Rotation2d signedAngleDifference(Rotation2d angle1, Rotation2d angle2) {
    double difference = (angle2.getDegrees() - angle1.getDegrees()) % 360;
    if (difference < -180) {
      difference += 360;
    } else if (difference >= 180) {
      difference -= 360;
    }
    return Rotation2d.fromDegrees(difference);
  }
}