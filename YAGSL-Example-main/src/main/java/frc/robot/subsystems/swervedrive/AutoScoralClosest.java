
package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class AutoScoralClosest extends Command {
    private Pose2d pose;
    private boolean isRight;
    private SwerveCommands swerveCommands;
    private IntakeSubsystem intake;
    private IntakeCommands intakeCommands;
    private ElevatorSubsystem elevatorSubsystem;
    private SwerveSubsystem swerve;

    public AutoScoralClosest(SwerveSubsystem swerve, SwerveCommands swerveCommands, IntakeSubsystem intake,
            IntakeCommands intakeCommands, ElevatorSubsystem elevatorSubsystem, boolean isRight) {
        this.isRight = isRight;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
    }

    @Override
    public void execute() {
        swerveCommands.autoScoral(pose, Constants.SetpointConstants.REEF_L2, isRight);
    }

    @Override
    public boolean isFinished() {
        return intake.isOut();
    }

    public Command autoAlign(double dist, boolean isRight) {
        var command = Commands.run(() -> {
            Pose2d targetPose = swerveCommands.addScoringOffset(pose, dist, isRight);// .55
            swerveCommands.actuallyMoveTo(targetPose);
        }).until(() -> {
            return Math.abs(pose.getX() - swerve.getPose().getX()) < 0.0508 &&
                    Math.abs(pose.getY() - swerve.getPose().getY()) < 0.0508 &&
                    Math.abs(pose.getRotation().getDegrees() - swerve.getPose().getRotation().getDegrees()) < 5;
        });
        command.addRequirements(swerve);
        return command;
    }

    public Command autoScoral(double setpoint, boolean isRight) { // put in desired pose and elevator
        // subsystem
        swerve.centerModulesCommand();
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                autoAlign(.8, isRight),
                swerveCommands.stopMoving(),
                elevatorSubsystem.goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                autoAlign(.32, isRight),
                swerveCommands.stopMoving(),
                intakeCommands.intakeOut());

        command.setName("autoScoral");
        // command.addRequirements(swerve, elevatorSubsystem);

        return command;
    }
}
