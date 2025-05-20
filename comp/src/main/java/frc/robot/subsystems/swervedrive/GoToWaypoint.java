// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class GoToWaypoint extends Command {

    private final SwerveSubsystem swerve;
    private final SwerveCommands swerveCommands;
    private double distLeeway = .0508;
    private double distance;
    private Pose2d waypoint;
    private Optional<Integer> tagNumber;
    private double diffX;
    private double diffY;
    private Rotation2d angle;
    private boolean isRight;

    public GoToWaypoint(Optional<Integer> tagNumber, SwerveSubsystem swerve, SwerveCommands swerveCommands) {
        this.tagNumber = tagNumber;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        addRequirements(swerve);
    }

    public GoToWaypoint(Optional<Integer> tagNumber, SwerveSubsystem swerve, SwerveCommands swerveCommands,
            boolean isRight) {
        this.tagNumber = tagNumber;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.isRight = isRight;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (tagNumber.isPresent()) {
            waypoint = swerveCommands.addScoringOffset(Vision.getTagPose(tagNumber.get()), 1, isRight);
        } else {
            waypoint = swerveCommands.addScoringOffset(
                    Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose())), 1,
                    isRight);
        }
    }

    @Override
    // public void execute() {

    // ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(diffX, diffY, angle);
    // swerve.driveFieldOriented(desiredSpeeds);
    // // swerveDrive.drive(new Translation2d(Math.pow(diffX, 3) *
    // Constants.MAX_SPEED,
    // // Math.pow(diffY, 3) * Constants.MAX_SPEED),
    // // Math.pow(angle, 3) * swerveDrive.getMaximumChassisAngularVelocity(),
    // // true,
    // // false);
    // }

    public void execute() {
        Translation2d translationDiff = waypoint.getTranslation().minus(swerve.getPose().getTranslation());
        diffX = translationDiff.getX() / translationDiff.getNorm();
        diffY = translationDiff.getY() / translationDiff.getNorm();

        angle = new Rotation2d(Math.atan2(diffX, diffY));

        distance = waypoint.getTranslation().getDistance(swerve.getPose().getTranslation());

        double percent = MathUtil.interpolate(.75, 1, distance / 2);
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(diffX * percent, diffY * percent, waypoint.getRotation());

        swerve.driveFieldOriented(desiredSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return distance < distLeeway;
    }

}
