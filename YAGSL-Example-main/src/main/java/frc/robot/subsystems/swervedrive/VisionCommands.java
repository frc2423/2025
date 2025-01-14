
package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterAngleCommands;
import frc.robot.subsystems.shooter.ShooterCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionCommands {
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;
    private IntakeSubsystem intake;
    private IntakeCommands intakeCommands;
    private ShooterCommands shooterCommands;
    private ShooterAngleCommands shooterAngleCommands;

    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase, IntakeSubsystem intake,
            IntakeCommands intakeCommands, ShooterCommands shooterCommands, ShooterAngleCommands shooterAngleCommands) {
        this.vision = vision;
        this.drivebase = drivebase;
        this.intake = intake;
        this.intakeCommands = intakeCommands;
        this.shooterAngleCommands = shooterAngleCommands;
        this.shooterCommands = shooterCommands;
    }

    public Command notePoseAutoAlign() {
        Command command = Commands.run(() -> drivebase.actuallyLookAngle(vision.getTurn()))
                .until(() -> vision.isAlignedNote());
        command.setName("note auto rotate align pose");
        return command;
    }

    public Command noteAutoAlign() {
        Command turn = Commands.run(() -> drivebase.turn(-vision.getNoteYaw() * .13), drivebase, vision);
        Command command = turn.until(() -> vision.isAlignedNote());
        command.setName("note auto rotate align");
        return command;
    }

    public Command noteAutoAlignPickUp() {
        Command turn = Commands.run(() -> drivebase.turn(-vision.getNoteYaw() * .075), drivebase, vision);
        Command driveAndTurn = Commands.run(() -> drivebase.turnAndGo(2, vision.yawAfterAligned()), drivebase, vision);
        Command stopMoving = Commands.runOnce(() -> drivebase.turnAndGo(0, 0), drivebase);
        Command command = Commands.sequence(
                Commands.parallel(intakeCommands.intakeDown(), shooterAngleCommands.handOffAngleCommand())
                        .withTimeout(.2),
                turn.until(() -> vision.isAlignedNote()),
                // intakeCommands.intakeDown(),
                Commands.parallel(driveAndTurn, intakeCommands.intakeIntake()).until(() -> intake.isBeamBroken()),
                stopMoving);
        return command;
    }

    public Command noteAutoAlignPickUpHandoffAndPrepareToShoot() {
        Command turn = Commands.run(() -> drivebase.turn(-vision.getNoteYaw() * .075), drivebase, vision);
        Command driveAndTurn = Commands.run(() -> drivebase.turnAndGo(2, vision.yawAfterAligned()), drivebase, vision);
        Command drive = drivebase.getTeleopDriveCommand();
        Command drive1 = drivebase.getTeleopDriveCommand();

        Command command = Commands.sequence(
                Commands.parallel(intakeCommands.intakeDown(), shooterAngleCommands.handOffAngleCommand())
                        .withTimeout(.2),
                Commands.parallel(
                        Commands.sequence(
                                drive.until(() -> vision.seesNote()),
                                turn.until(() -> vision.isAlignedNote()),
                                driveAndTurn.until(() -> intake.isBeamBroken()), drive1),
                        Commands.sequence(
                                intakeCommands.intakeIntake().until(() -> intake.isBeamBroken()),
                                shooterCommands.intakeSequencePlusHandoffCommand(),
                                intakeCommands.beltStopCommand(),
                                Commands.runOnce(() -> intake.getCurrentCommand().cancel()))));
        command.setName("Note Align Pick Up Handoff and PrepareToShoot");
        return command;
    }
}
