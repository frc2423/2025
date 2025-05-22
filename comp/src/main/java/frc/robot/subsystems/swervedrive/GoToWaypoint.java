// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.util.Optional;
import java.util.function.Supplier;

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
    private Optional<Integer> tagNumber = Optional.empty();
    private double diffX;
    private double diffY;
    private boolean isRight;
    private double dist = .6;
    private Optional<Supplier<Pose2d>> poseSupplier = Optional.empty();

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

    public GoToWaypoint(Optional<Integer> tagNumber, RobotContainer container,
            boolean isRight, double dist) {
        this.tagNumber = tagNumber;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.levelPicker = container.elevatorLevelPicker;
        this.isRight = isRight;
        this.dist = dist;
        addRequirements(swerve);
    }

    public GoToWaypoint(RobotContainer container, Supplier<Pose2d> poseSupplier) {
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.levelPicker = container.elevatorLevelPicker;
        this.addRequirements(swerve);
        this.poseSupplier = Optional.of(poseSupplier);
    }

    @Override
    public void initialize() {
        if (tagNumber.isPresent()) {
            waypoint = swerveCommands.addScoringOffset(Vision.getTagPose(tagNumber.get()), dist, isRight);
        } else if (!this.poseSupplier.isEmpty()) {
            waypoint = this.poseSupplier.get().get();
        } else {
            waypoint = swerveCommands.addScoringOffset(levelPicker.getNearestOpenReefPose(), dist, isRight);
        }
    }

    @Override
    public void execute() {
        Translation2d translationDiff = waypoint.getTranslation().minus(swerve.getPose().getTranslation());
        diffX = translationDiff.getX() / translationDiff.getNorm();
        diffY = translationDiff.getY() / translationDiff.getNorm();

        distance = waypoint.getTranslation().getDistance(swerve.getPose().getTranslation());

        double percent = MathUtil.interpolate(.5, 1, distance / 1.2);
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
        return distance < distLeeway;
    }

}
