package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

import au.grapplerobotics.LaserCan;

public class IntakeCommands {
    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command intakeIn() {
        var command = Commands.run(() -> {
            intake.intake(0.15);
        }).until(() -> intake.distMm() < 100).andThen(intakeStop());
        command.setName("Intake In");
        command.addRequirements(intake);
        return command;
    }

    public Command intakeOut() {
        var command = Commands.run(() -> {
            intake.intake(0.15);
        }).until(() -> intake.distMm() > 100).andThen(intakeStop());
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