package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PoseTransformUtils {

    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.21;

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        return Alliance.Red.equals(DriverStation.getAlliance().get());
    }

    public static Pose2d transformRedPose(Pose2d startPose) {
        if (isRedAlliance()) {
            double realX = FIELD_LENGTH_METERS - startPose.getX();
            double realY = FIELD_WIDTH_METERS - startPose.getY();
            Rotation2d realANGLE = startPose.getRotation().plus(new Rotation2d(Math.PI));
            Pose2d transformedPose = new Pose2d(realX, realY, realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }

    public static Pose3d transformRedPose(Pose3d startPose) {
        if (isRedAlliance()) {
            double realX = FIELD_LENGTH_METERS - startPose.getX();
            double realY = FIELD_WIDTH_METERS - startPose.getY();
            Rotation3d realANGLE = startPose.getRotation().plus(new Rotation3d(0, 0, Math.PI));
            Pose3d transformedPose = new Pose3d(realX, realY, startPose.getZ(), realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }

    public static Pose2d transformYRedPose(Pose2d startPose) {
        if (isRedAlliance()) {
            double realX = startPose.getX();
            double realY = FIELD_WIDTH_METERS - startPose.getY();
            Rotation2d realANGLE = startPose.getRotation().plus(new Rotation2d(Math.PI));
            Pose2d transformedPose = new Pose2d(realX, realY, realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }

    public static Pose2d transformXRedPose(Pose2d startPose) {
        if (isRedAlliance()) {
            double realX = FIELD_LENGTH_METERS - startPose.getX();
            double realY = startPose.getY();
            Rotation2d realANGLE = startPose.getRotation().plus(new Rotation2d(Math.PI));
            Pose2d transformedPose = new Pose2d(realX, realY, realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }

    public static Pose3d transformYRedPose(Pose3d startPose) {
        if (isRedAlliance()) {
            double realX = startPose.getX();
            double realY = FIELD_WIDTH_METERS - startPose.getY();
            Rotation3d realANGLE = startPose.getRotation().plus(new Rotation3d(0, 0, Math.PI));
            Pose3d transformedPose = new Pose3d(realX, realY, startPose.getZ(), realANGLE);
            return transformedPose;
        } else {
            return startPose;
        }
    }
}