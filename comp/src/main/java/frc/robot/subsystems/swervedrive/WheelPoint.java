// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class WheelPoint extends Command {

  private final SwerveSubsystem swerve;
  private double dist;
  private Vision vision;
  private SwerveCommands swerveCommands;
  private Translation2d targetPose;
  private Translation2d initialPosition;
  private double speed;

  public WheelPoint(SwerveSubsystem swerve, double dist, Pose2d pose, Translation2d targetPosition) {
    this.swerve = swerve;
    this.dist = dist;
    Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, true);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    double X1 = swerve.getPose().getX();
    double Y1 = swerve.getPose().getY();
    double X2 = targetPose.getX();
    double Y2 = targetPose.getY();
    // double math = (Y2 - Y1) / (X2 - X1);
    double angle = Math.atan2(Y2 - Y1, X2 - X1);
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
