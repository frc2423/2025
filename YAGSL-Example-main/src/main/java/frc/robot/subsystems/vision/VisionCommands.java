package frc.robot.subsystems.vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommands {
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;

    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase) {
        this.vision = vision;
        this.drivebase = drivebase;
    }
    
}