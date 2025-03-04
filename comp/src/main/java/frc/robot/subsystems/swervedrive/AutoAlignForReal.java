package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.AngleUtils;
import frc.robot.NTHelper;

public class AutoAlignForReal extends Command {
    private Pose2d pose;
    private boolean isRight;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private boolean reachedY = false;
    private boolean reachedX = false;

    private final static int FILTER_SIZE = 10;
    MedianFilter xDistanceFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter yDistanceFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter targetAngleFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter swerveAngleFilter = new MedianFilter(FILTER_SIZE);

    public AutoAlignForReal(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
    }

    @Override
    public void initialize() {
        reachedY = false;
        reachedX = false;
        pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
        xDistanceFilter.reset();
        yDistanceFilter.reset();
        targetAngleFilter.reset();
        swerveAngleFilter.reset();
        for (double i = 0; i < FILTER_SIZE; i++) {
            xDistanceFilter.calculate(100000);
            yDistanceFilter.calculate(100000);
            targetAngleFilter.calculate(100000);
            swerveAngleFilter.calculate(100000);
        }
    }

    @Override
    public void execute() {
        Pose2d pose2d = swerveCommands.addScoringOffset(pose, dist, isRight);// .55
        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());

        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        double xSign = Math.copySign(1, translationDiff.getX());
        double ySign = Math.copySign(1, translationDiff.getY());

        double x = 0;
        double y = 0;

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        if (reachedY) {
            y = 0;
        } else if (yDistance > .7) {
            y = Math.copySign(.7, ySign);
        } else if (yDistance > .3) {
            y = Math.copySign(.65, ySign);
        } else {
            y = Math.copySign(.55, ySign);
        }

        if (reachedY) {
            if (reachedX) {
                x = 0;
            } else if (xDistance > .7) {
                x = Math.copySign(.7, xSign);
            } else if (xDistance > .3) {
                x = Math.copySign(.65, xSign);
            } else {
                x = Math.copySign(.55, xSign);
            }
        }

        if (yDistance < Units.inchesToMeters(2))
            reachedY = true;
        if (xDistance < Units.inchesToMeters(2))
            reachedX = true;

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(x, y, pose2d.getRotation());

        final double maxRadsPerSecond = 5;

        if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond) {
            desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond,
                    desiredSpeeds.omegaRadiansPerSecond);
        }

        swerve.drive(desiredSpeeds);
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55

        // double averageXDistance =
        // xDistanceFilter.calculate(Math.abs(targetPose.getX() -
        // swerve.getPose().getX()));
        // double averageYDistance =
        // yDistanceFilter.calculate(Math.abs(targetPose.getY() -
        // swerve.getPose().getY()));
        double averageTargetPose = targetAngleFilter.calculate(targetPose.getRotation().getRadians());
        double averageSwervePose = swerveAngleFilter.calculate(swerve.getPose().getRotation().getRadians());
        boolean isAngleClose = AngleUtils.areAnglesClose(new Rotation2d(averageTargetPose),
                new Rotation2d(averageSwervePose),
                Rotation2d.fromDegrees(4));

        // return averageXDistance < Units.inchesToMeters(2) &&
        // averageYDistance < Units.inchesToMeters(2) &&
        // isAngleClose;
        return reachedY && reachedX && isAngleClose;
    }
}
