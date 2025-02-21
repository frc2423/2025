
package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.NTHelper;

public class AutoAlignClosest extends Command {
    private Pose2d pose;
    private boolean isRight;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;

    public AutoAlignClosest(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
    }

    @Override
    public void initialize() {
        pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
    }

    @Override
    public void execute() {
        Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55
        swerveCommands.actuallyMoveTo(targetPose);
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55

        double xDistance = Math.abs(targetPose.getX() - swerve.getPose().getX());
        double yDistance = Math.abs(targetPose.getY() - swerve.getPose().getY());
        double angleDistance = Math
                .abs(targetPose.getRotation().minus(swerve.getPose().getRotation())
                        .getDegrees());
        return xDistance < 0.0508 &&
                yDistance < 0.0508 &&
                angleDistance < 5;
    }
}
