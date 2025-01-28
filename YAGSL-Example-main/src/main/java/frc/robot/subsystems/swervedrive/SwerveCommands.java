package frc.robot.subsystems.swervedrive;

import java.awt.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

import java.util.ArrayList;
import java.util.Collections;


public class SwerveCommands {
    XboxController driverXbox = new XboxController(0);
    private SwerveSubsystem swerve;
    private Vision vision;

    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command lookAtAngle(double angle){
        var command = Commands.run(() -> {
            swerve.actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));
        }).until(() -> (driverXbox.getRightX() > .1) || (driverXbox.getRightY() > .1));
        command.addRequirements(swerve);
        return command;
    }

    public Command autoZoneAlign(double closestAngle){
        ArrayList distanceToAprilTag = new ArrayList();
        for (int i = 1;i < 22; i++) {
            distanceToAprilTag.add(vision.getDistanceFromAprilTag(i));
        }
        Collections.sort(distanceToAprilTag);
        distanceToAprilTag.get(0); 
        var command = Commands.run(() -> {
            swerve.actuallyLookAngleButMove(Rotation2d.fromDegrees(closestAngle));
        });
        command.addRequirements(swerve);
        return command;
    }
    
}
