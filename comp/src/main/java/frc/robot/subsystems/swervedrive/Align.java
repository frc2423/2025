package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleUtils;

public class Align extends Command {
    private Pose2d pose;
    private boolean isRight;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;

    private boolean reachedX = false;
    private boolean reachedY = false;

    public Align(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
    }

    @Override
    public void initialize() {
        pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
        reachedX = false;
        reachedY = false;
    }

    @Override
    public void execute() {
        Pose2d pose2d = swerveCommands.addScoringOffset(pose, dist, isRight);// .55

        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());
        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        double xSign = Math.copySign(1, translationDiff.getX());
        double ySign = Math.copySign(1, translationDiff.getY());

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        double x = 0;
        double y = 0;

        if (reachedX) {
            x = 0;
        } else if (xDistance > .8) {
            x = Math.copySign(1, xSign);
        } else if (xDistance > .2) {
            x = Math.copySign(.6, xSign);
        } else {
            x = Math.copySign(.4, xSign);
        }

        if (reachedY) {
            y = 0;
        } else if (yDistance > .8) {
            y = Math.copySign(1, ySign);
        } else if (yDistance > .2) {
            y = Math.copySign(.6, ySign);
        } else {
            y = Math.copySign(.4, ySign);
        }

        if (xDistance < Units.inchesToMeters(2)) {
            reachedX = true;
        }
        if (yDistance < Units.inchesToMeters(2)) {
            reachedY = true;
        }

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
        return reachedX && reachedY;
    }
}
