
package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AngleUtils;
import frc.robot.PoseTransformUtils;

public class AutoAlignProcessor extends Command {
    private double distance;
    private double diffX;
    private double diffY;
    private Pose2d waypoint;
    private Pose2d pose;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private double offsetY = 0;
    static Pose2d ProcessorPoseBLUE = new Pose2d(new Translation2d(5.982, 0.567), Rotation2d.fromDegrees(-90));
    static Pose2d ProcessorPoseRED = new Pose2d(new Translation2d(11.520, 7.543), Rotation2d.fromDegrees(90));

    // public AutoAlignProcessor(SwerveSubsystem swerve, SwerveCommands
    // swerveCommands, Optional<Boolean> isRight) {
    // this.isRight = isRight;
    // this.swerve = swerve;
    // this.swerveCommands = swerveCommands;
    // this.addRequirements(swerve);
    // }

    public AutoAlignProcessor(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist) {
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.addRequirements(swerve);
    }

    public String isRedAlliance() {
        if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
            return "RED";
        } else {
            return "BLUE";
        }
    }

    @Override
    public void initialize() {
        if (isRedAlliance().equals("RED")) {
            pose = ProcessorPoseRED;

        } else {
            pose = ProcessorPoseBLUE;
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
