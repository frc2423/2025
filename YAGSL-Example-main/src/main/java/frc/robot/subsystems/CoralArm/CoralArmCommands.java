package frc.robot.subsystems.CoralArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CoralArmCommands {
    private CoralArmSubSystem coralarm;

    public CoralArmCommands(CoralArmSubSystem coralarm) {
        this.coralarm = coralarm;
    }
    public Command goForward() {
        var command = Commands.run(() -> coralarm.goForward());
        return command;
    }

    public Command stop() {
        var command = Commands.run(() -> coralarm.stop());
        return command;
    }
    }
