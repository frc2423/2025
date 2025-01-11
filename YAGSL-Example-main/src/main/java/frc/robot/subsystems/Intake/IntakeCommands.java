package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import au.grapplerobotics.LaserCan;

public class IntakeCommands {
    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command intakeIn() {
        var command = Commands.run(() -> {
            //if (intake.distMm() > 5) {
                intake.intake(0.3);
            //} else {
                //intake.stop();
            //}
        });
        command.setName("Intake In");
        command.addRequirements(intake);
        return command;
    }

    public Command intakeOut() {
        var command = Commands.run(() -> intake.outtake(-0.1));
        command.addRequirements(intake);
        command.setName("Intake Out");
        return command;
    }

    public Command intakeStop() {
        var command = Commands.run(() -> intake.stop());
        command.addRequirements(intake);
        command.setName("Intake Stop");
        return command;
    }
}