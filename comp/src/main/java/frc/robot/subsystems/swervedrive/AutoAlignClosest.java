
package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleUtils;
import frc.robot.NTHelper;

public class AutoAlignClosest extends Command {
    private Pose2d pose;
    private boolean isRight;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;

    MedianFilter xDistanceFilter = new MedianFilter(5);
    MedianFilter yDistanceFilter = new MedianFilter(5);
    MedianFilter targetAngleFilter = new MedianFilter(5);
    MedianFilter swerveAngleFilter = new MedianFilter(5);

    public AutoAlignClosest(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
    }

    @Override
    public void initialize() {
        pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
        xDistanceFilter.reset();
        yDistanceFilter.reset();
        targetAngleFilter.reset();
        swerveAngleFilter.reset();
        for (double i = 0; i < 5; i++) {
            xDistanceFilter.calculate(100000);
            yDistanceFilter.calculate(100000);
            targetAngleFilter.calculate(100000);
            swerveAngleFilter.calculate(100000);
        }

    }

    @Override
    public void execute() {
        Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55
        swerveCommands.actuallyMoveTo(targetPose);
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55

        double averageXDistance = xDistanceFilter.calculate(Math.abs(targetPose.getX() - swerve.getPose().getX()));
        double averageYDistance = yDistanceFilter.calculate(Math.abs(targetPose.getY() - swerve.getPose().getY()));
        double averageTargetPose = targetAngleFilter.calculate(targetPose.getRotation().getRadians());
        double averageSwervePose = swerveAngleFilter.calculate(swerve.getPose().getRotation().getRadians());
        boolean isAngleClose = AngleUtils.areAnglesClose(new Rotation2d(averageTargetPose),
                new Rotation2d(averageSwervePose),
                Rotation2d.fromDegrees(1));

        return averageXDistance < 0.0508 &&
                averageYDistance < 0.0508 &&
                isAngleClose;
    }
}
