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

    public IntakeCommands(IntakeSubsystem intake, FunnelSubsystem funnel) {
        this.intake = intake;
        this.funnel = funnel;
    }

    public Command intakeIn() {
        var command = Commands.run(() -> {
            // arm.goUp(),
            intake.intake(.15);
        });
        command.setName("Intake In");
        command.addRequirements(intake);
        return command;
    }

    public Command intakeOut() {
        var command = Commands.run(() -> {
            intake.intake(0.3);
        }).until(() -> intake.isOut()).andThen(intakeStop());
        command.addRequirements(intake);
        command.setName("Intake Out");
        return command;
    }

    public Command intakeStop() {
        var command = Commands.runOnce(() -> intake.stop());
        command.addRequirements(intake);
        command.setName("Intake Stop");
        return command;
    }

    public Command in() {
        var command = Commands.parallel(intakeIn(), funnel.spinInBoth()).until(() -> !intake.isOut())
                .andThen(stop());
        command.addRequirements(intake);
        command.addRequirements(funnel);
        command.setName("In");
        return command;
    }

    public Command stop() {
        var command = Commands.sequence(
                intakeStop(),
                funnel.stop());
        command.addRequirements(intake);
        command.addRequirements(funnel);
        command.setName("stop both funnel and intake");
        return command;
    }
}