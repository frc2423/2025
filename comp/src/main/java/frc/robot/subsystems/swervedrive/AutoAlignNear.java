package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.AngleUtils;

public class AutoAlignNear extends Command {
    private Pose2d pose;
    private boolean isRight;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private boolean reachedY = false;
    private boolean reachedX = false;
    private Optional<Integer> tagNumber = Optional.empty();

    private final static int FILTER_SIZE = 10;
    MedianFilter targetAngleFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter swerveAngleFilter = new MedianFilter(FILTER_SIZE);

    public AutoAlignNear(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.addRequirements(swerve);
    }

    public AutoAlignNear(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight,
            Optional<Integer> tagNumber) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.tagNumber = tagNumber;
        this.addRequirements(swerve);
    }

    @Override
    public void initialize() {
        reachedY = false;
        reachedX = false;
        if (tagNumber.isPresent()) {
            pose = Vision.getTagPose(tagNumber.get());
        } else {
            pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
        }
        targetAngleFilter.reset();
        swerveAngleFilter.reset();
        for (double i = 0; i < FILTER_SIZE; i++) {
            targetAngleFilter.calculate(100000);
            swerveAngleFilter.calculate(100000);
        }
    }

    @Override
    public void execute() {
        Pose2d pose2d = swerveCommands.addScoringOffset(pose, dist, isRight);// .55
        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());

        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        if (yDistance < Units.inchesToMeters(.8)) {
            reachedY = true;
        }
        if (xDistance < Units.inchesToMeters(.8)) {
            reachedX = true;
        }

        swerveCommands.actuallyMoveTo(pose2d, reachedY && !reachedX, !reachedY);
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55

        double averageTargetPose = targetAngleFilter.calculate(targetPose.getRotation().getRadians());
        double averageSwervePose = swerveAngleFilter.calculate(swerve.getPose().getRotation().getRadians());
        boolean isAngleClose = AngleUtils.areAnglesClose(new Rotation2d(averageTargetPose),
                new Rotation2d(averageSwervePose),
                Rotation2d.fromDegrees(3));

        return reachedY && reachedX && isAngleClose;
    }
}
