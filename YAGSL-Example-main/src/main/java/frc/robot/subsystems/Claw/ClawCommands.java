package frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class ClawCommands {
    private ClawSubsystem claw;

    public ClawCommands(ClawSubsystem claw) {
        this.claw = claw;
    }

    public Command clawRelease() {
        var command = Commands.run(() -> claw.release());
        command.setName("Claw Release");
        return command;
}
    public Command clawStop() {
        var command = Commands.run(() -> claw.stop());
        command.setName("Claw Stop");
        return command;
}
}
