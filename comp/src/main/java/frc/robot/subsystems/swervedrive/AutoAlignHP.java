
package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleUtils;
import frc.robot.PoseTransformUtils;

public class AutoAlignHP extends Command {
    private double distance;
    private double diffX;
    private double diffY;
    private Pose2d waypoint;
    private Pose2d pose;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private Optional<Boolean> isRight = Optional.empty();
    private double offsetY = 0;

    public AutoAlignHP(SwerveSubsystem swerve, SwerveCommands swerveCommands, Optional<Boolean> isRight) {
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

    public int isItRight() {
        if (isRight.isPresent()) {
            if (isRight.get() == true) {
                if (PoseTransformUtils.isRedAlliance()) {
                    return 2;
                } else {
                    return 12;
                }
            } else {
                if (PoseTransformUtils.isRedAlliance()) {
                    return 1;
                } else {
                    return 13;
                }
            }
        }
        return 0;
    }

    @Override
    public void initialize() {
        if (isRight.isPresent()) {
            pose = Vision.getTagPose(isItRight());
        } else {
            pose = Vision.getTagPose(swerve.vision.findClosestHPSTagID(swerve.getPose()));
        }

        waypoint = swerveCommands.addOffset(pose, dist, offsetY);
    }

    @Override
    public void execute() {
        Translation2d translationDiff = waypoint.getTranslation().minus(swerve.getPose().getTranslation());
        diffX = translationDiff.getX() / translationDiff.getNorm();
        diffY = translationDiff.getY() / translationDiff.getNorm();

        System.out.println(diffX * diffX + diffY * diffY);

        distance = waypoint.getTranslation().getDistance(swerve.getPose().getTranslation());

        double percent = MathUtil.interpolate(.5, 1, distance / 2);
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeedsUnscaled(diffX * percent, diffY * percent,
                waypoint.getRotation());

        swerve.driveFieldOriented(desiredSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return distance < Units.inchesToMeters(2);
    }
}
