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

    public void autoScoral(Pose2d pose, double setpoint) { // put in desired pose and elevator subsystem
        Pose2d pose1 = new Pose2d();
        Pose2d pose2 = new Pose2d();
        Transform2d poseDiff = pose1.minus(pose2);
        Command command1 = autoAlign(new Pose2d()).until(() -> {
            Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);
            double distance = Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2)); // distance in meters

            if (distance <= .0833) {
                return true;
            } else {
                return false;
            }
        }).andThen(elevatorSubsystem.goToSetpoint(setpoint)).andThen(intakeCommands.intakeOut());

        // Command command2 = ElevatorSubsystem.goToSetpoint().until(() -> {
        // if(position = ){
        // return true;
        // }
        // return false;
        // });
        // Command command3 = IntakeCommands.intakeOut()
    }
}
