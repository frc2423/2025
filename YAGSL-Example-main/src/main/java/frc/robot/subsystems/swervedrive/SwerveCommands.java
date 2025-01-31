package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PoseTransformUtils;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SwerveCommands {

    private SwerveSubsystem swerve;
    private IntakeCommands intakeCommands;
    private ElevatorSubsystem elevatorSubsystem;

    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command autoAlign(Pose2d pose) {

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                0.5, 1.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0 // Goal end velocity in meters/sec
        );// Rotation delay distance in meters. This is how far the robot should travel
          // before attempting to rotate.

        pathfindingCommand.setName("Align to Pose");

        return pathfindingCommand;
    }

    public Command autoScoral(Pose2d pose, double setpoint) { // put in desired pose and elevator subsystem
        return autoAlign(pose).until(() -> {
            Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);
            Pose2d robotPose = swerve.getPose();
            Transform2d poseDiff = targetPose.minus(robotPose);
            double distance = Math.sqrt(Math.pow(poseDiff.getX(), 2) + Math.pow(poseDiff.getY(), 2)); // distance in
                                                                                                      // meters
            return distance <= .0833;
        }).andThen(elevatorSubsystem.goToSetpoint(setpoint)).until(() -> {
            return elevatorSubsystem.isAtSetpoint();
        }).andThen(intakeCommands.intakeOut());
    }
}
