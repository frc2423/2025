package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AngleUtils;

public class AutoAlignPose extends Command {
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private Pose2d targetPose;

    private final static int FILTER_SIZE = 10;
    MedianFilter xDistanceFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter yDistanceFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter targetAngleFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter swerveAngleFilter = new MedianFilter(FILTER_SIZE);

    public AutoAlignPose(SwerveSubsystem swerve, SwerveCommands swerveCommands, Pose2d targetPose) {
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.targetPose = targetPose;
        this.addRequirements(swerve);
    }

    @Override
    public void initialize() {
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
        swerveCommands.actuallyMoveTo(targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        double averageXDistance = xDistanceFilter.calculate(Math.abs(targetPose.getX() - swerve.getPose().getX()));
        double averageYDistance = yDistanceFilter.calculate(Math.abs(targetPose.getY() - swerve.getPose().getY()));
        double averageTargetPose = targetAngleFilter.calculate(targetPose.getRotation().getRadians());
        double averageSwervePose = swerveAngleFilter.calculate(swerve.getPose().getRotation().getRadians());
        boolean isAngleClose = AngleUtils.areAnglesClose(new Rotation2d(averageTargetPose),
                new Rotation2d(averageSwervePose),
                Rotation2d.fromDegrees(4));

        return averageXDistance < Units.inchesToMeters(2) &&
                averageYDistance < Units.inchesToMeters(2) &&
                isAngleClose;
    }
}
