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
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorLevelPicker;

public class GoToWaypoint extends Command {

    private final SwerveSubsystem swerve;
    private final SwerveCommands swerveCommands;
    private final ElevatorLevelPicker levelPicker;
    private double distLeeway = .0508;
    private double distance;
    private Pose2d waypoint;
    private Optional<Integer> tagNumber;
    private double diffX;
    private double diffY;
    private Rotation2d angle;
    private boolean isRight;

    public GoToWaypoint(Optional<Integer> tagNumber, RobotContainer container) {
        this.tagNumber = tagNumber;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.levelPicker = container.elevatorLevelPicker;
        addRequirements(swerve);
    }

    public GoToWaypoint(Optional<Integer> tagNumber, RobotContainer container,
            boolean isRight) {
        this.tagNumber = tagNumber;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.levelPicker = container.elevatorLevelPicker;
        this.isRight = isRight;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (tagNumber.isPresent()) {
            waypoint = swerveCommands.addScoringOffset(Vision.getTagPose(tagNumber.get()), .6, isRight);
        } else {
            waypoint = swerveCommands.addScoringOffset(levelPicker.getNearestOpenReefPose(), .6, isRight);
        }
    }

    @Override
    public void execute() {
        Translation2d translationDiff = waypoint.getTranslation().minus(swerve.getPose().getTranslation());
        diffX = translationDiff.getX() / translationDiff.getNorm();
        diffY = translationDiff.getY() / translationDiff.getNorm();

        angle = new Rotation2d(Math.atan2(diffX, diffY));

        distance = waypoint.getTranslation().getDistance(swerve.getPose().getTranslation());

        double percent = MathUtil.interpolate(.5, 1, distance / 2);
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
