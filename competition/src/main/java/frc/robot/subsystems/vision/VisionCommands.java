package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionCommands {
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;

    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase) {
        this.vision = vision;
        this.drivebase = drivebase;
    }
    
}