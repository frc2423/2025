package frc.robot.subsystems.Intake;

//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;

import au.grapplerobotics.LaserCan;

public class IntakeCommands {
    private IntakeSubsystem intake;
    private FunnelSubsystem funnel;
    // private ArmSubsystem arm;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    // public Command intakeIn() {
    // var command = Commands.sequence(
    // arm.goUp(),
    // in());
    // command.setName("Intake In");
    // command.addRequirements(intake);
    // return command;
    // }

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

    public Command in() {
        var command = Commands.run(() -> {
            funnel.spinInBoth();
            intake.intake(0.15);
        }).until(() -> intake.distMm() < 100).andThen(intakeStop());
        return command;
    }
}