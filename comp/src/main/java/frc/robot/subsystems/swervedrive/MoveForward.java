// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveForward extends Command {

  private final SwerveSubsystem swerve;
  private double dist;
  private Translation2d targetPosition;
  private Translation2d initialPosition;
  private double speed;

  public MoveForward(SwerveSubsystem swerve, double dist, double speed) {
    this.swerve = swerve;
    this.dist = dist;
    this.speed = speed;
    addRequirements(swerve);
  }

  // public MoveForward(SwerveSubsystem swerve, Translation2d targetPosition,
  // double speed) {
  // this.swerve = swerve;
  // this.targetPosition = targetPosition;
  // this.speed = speed;
  // addRequirements(swerve);
  // }

  @Override
  public void initialize() {
    initialPosition = swerve.getPose().getTranslation();
    // if (targetPosition != null) {
    // dist = targetPosition.getDistance(initialPosition);
    // }
  }

  @Override
  public void execute() {
    swerve.drive(new ChassisSpeeds(speed, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return initialPosition.getDistance(swerve.getPose().getTranslation()) > dist;
  }

}
