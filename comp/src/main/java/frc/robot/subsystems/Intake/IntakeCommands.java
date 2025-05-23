package frc.robot.subsystems.Intake;

//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.BooleanSupplier;

import au.grapplerobotics.LaserCan;

public class IntakeCommands {
    private IntakeSubsystem intake;
    private FunnelSubsystem funnel;
    // private ArmSubsystem arm;

    public IntakeCommands(IntakeSubsystem intake, FunnelSubsystem funnel) {
        this.intake = intake;
        this.funnel = funnel;
        intake.setDefaultCommand(intakeStop());
    }

    public Command intakeIn() {
        var command = Commands.run(() -> {
            // arm.goUp(),
            intake.intake(.3); // .225
        });
        command.setName("Intake In");
        command.addRequirements(intake);
        return command;
    }

    public Command intakeStart() {
        var command = Commands.runOnce(() -> {
            intake.intake(.25);
        });
        command.setName("Intake Once");
        command.addRequirements(intake);
        return command;
    }

    public Command intakeShort() {
        return intakeHumanPlayer().withTimeout(0.5).withName("Run Intake Short");
    }

    public Command intakeJustIn() {
        var command = Commands.runOnce(() -> {
            intake.intake(.3);
        });
        command.addRequirements(intake);
        command.setName("Just IN");
        return command;
    }

    public Command intakeAlgae() {
        var command = Commands.run(() -> {
            intake.intake(1);
        });// .until(() -> intake.hasAlgae());
        command.addRequirements(intake);
        command.setName("algae intake");
        return command;
    }

    public Command holdAlgae() {
        var command = Commands.run(() -> {
            intake.intake(1);
        });
        command.addRequirements(intake);
        command.setName("algae intake");
        return command;
    }

    public Command intakeOut() {
        var command = Commands.run(() -> {
            intake.intake(1);
        }).until(() -> intake.isOutMedianFilter()).andThen(intakeStop());
        command.addRequirements(intake);
        command.setName("Intake Out");
        return command;
    }

    public Command intakeJustOut() {
        var command = Commands.runOnce(() -> {
            intake.intake(0.3);
        });
        command.addRequirements(intake);
        command.setName("Just Out");
        return command;
    }

    public Command intakeJustOutRun() {
        var command = Commands.run(() -> {
            intake.intake(1);
        });
        command.addRequirements(intake);
        command.setName("Just Out");
        return command;
    }

    public Command ejectAlgae() {
        var command = Commands.run(() -> {
            intake.outtakeAlgae(1);
        });
        command.addRequirements(intake);
        command.setName("Eject Algae");
        return command;
    }

    public Command eject() {
        Command intakeOut = Commands.runOnce(() -> {
            intake.backwards(.5);
        });
        var command = Commands.parallel(intakeOut, funnel.spinOutOnce());
        command.addRequirements(intake, funnel);
        command.setName("Eject");
        return command;
    }

    public Command intakeStop() {
        var command = Commands.runOnce(() -> intake.stop());
        command.addRequirements(intake);
        command.setName("Intake Stop");
        return command;
    }

    public Command in() {
        var command = Commands.parallel(intakeIn(), funnel.spinIn()).until(() -> !intake.isOut())
                .andThen(stop());
        command.addRequirements(intake);
        command.addRequirements(funnel);
        command.setName("In");
        return command;
    }

    public Command backwards() {
        var command = Commands.run(() -> intake.backwards(.15));
        command.setName("intake backwards");
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

    public Command intakeHumanPlayer() {
        var command = Commands.parallel(intakeIn(), funnel.spinIn()).until(() -> !intake.isOut())
                .andThen(stop());
        command.addRequirements(intake);
        command.addRequirements(funnel);
        command.setName("Intake Coral From Human Player");
        return command;
    }

    public BooleanSupplier hasNoCoral() {
        return () -> !intake.isOut();
    }

    public BooleanSupplier hasNoAlgae() {
        return () -> !intake.hasAlgae();
    }
}
