
package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleUtils;
import frc.robot.PoseTransformUtils;

public class AutoAlignHP extends Command {
    private Pose2d pose;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private Optional<Boolean> isRight = Optional.empty();
    private double offsetY = 0;

    private final static int FILTER_SIZE = 10;
    MedianFilter xDistanceFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter yDistanceFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter targetAngleFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter swerveAngleFilter = new MedianFilter(FILTER_SIZE);

    public AutoAlignHP(SwerveSubsystem swerve, SwerveCommands swerveCommands, Optional <Boolean> isRight) {
        this.isRight = isRight;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.addRequirements(swerve);
    }

    public AutoAlignHP(SwerveSubsystem swerve, SwerveCommands swerveCommands) {
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.addRequirements(swerve);
    }

    public int isItRight(){
        if(isRight.isPresent()){
            if(isRight.get() == true){
                if(PoseTransformUtils.isRedAlliance()){
                    return 2;
            }
            }
        }
        return 12;
    }

    @Override
    public void initialize() {
        if (tagNumber.isPresent()) {
            pose = Vision.getTagPose(isItRight());
        } else {
            pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
        }
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
        Pose2d targetPose = swerveCommands.addOffset(pose, dist, offsetY);// .55
        swerveCommands.actuallyMoveTo(targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = swerveCommands.addOffset(pose, dist, offsetY);// .55

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
