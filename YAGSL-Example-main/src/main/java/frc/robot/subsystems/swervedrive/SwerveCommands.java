package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.PoseTransformUtils;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.PoseTransformUtils;

public class SwerveCommands {

    private SwerveSubsystem swerve;
    private IntakeCommands intakeCommands;

    private ElevatorSubsystem elevatorSubsystem;
    private XboxController driverXbox = new XboxController(0);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(7);

    public SwerveCommands(SwerveSubsystem swerve, ElevatorSubsystem elevatorSubsystem,
            IntakeCommands intakeCommands) {
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeCommands = intakeCommands;

    }

    public Pose2d addScoringOffset(Pose2d pose, double distance) {
        Transform2d offset = new Transform2d(distance, 0, Rotation2d.kPi);
        Pose2d targetPose = pose.plus(offset);
        return targetPose;
    }

    public Command autoAlign(Pose2d pose) {

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        // Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);
        Pose2d targetPose = addScoringOffset(pose, 1);

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
        pathfindingCommand.addRequirements(swerve);

        return pathfindingCommand;
    }

    public Command autoScoral(Pose2d pose, double setpoint) { // put in desired pose and elevator subsystem

        var command = Commands.sequence(
                autoAlign(pose).until(() -> {
                    Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);
                    Pose2d robotPose = swerve.getPose();
                    Transform2d poseDiff = targetPose.minus(robotPose);
                    double distance = Math.sqrt(Math.pow(poseDiff.getX(), 2) +
                            Math.pow(poseDiff.getY(), 2)); // distance in
                    // meters
                    return distance <= .0833;
                }),
                elevatorSubsystem.goToSetpoint(setpoint).until(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                intakeCommands.intakeOut());

        // var command = autoAlign(pose).until(() -> {
        // return false;
        // }).andThen(elevatorSubsystem.goToSetpoint(setpoint)).until(() -> {
        // return elevatorSubsystem.isAtSetpoint();
        // });

        command.setName("autoScoral");
        // command.addRequirements(swerve, elevatorSubsystem);

        return command;
    }

    public Command lookAtAngle(double angle) {
        var command = Commands.run(() -> {
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));
        }).until(() -> (driverXbox.getRightX() > .1) || (driverXbox.getRightY() > .1));
        command.addRequirements(swerve);
        return command;
    }

    public Command lookAtNearestTag() {
        var command = Commands.run(() -> {
            int tag = swerve.vision.findClosestTagID(swerve.getPose());
            int angle = swerve.vision.iDtoAngle(tag);
            lookAtAngle(angle);
            System.out.println("Angle " + angle + " Tag " + tag);
        });

        return command;
    }

    public void actuallyLookAngleButMove(Rotation2d rotation2d) { // here
        final double maxRadsPerSecond = 5;

        double x = MathUtil.applyDeadband(
                driverXbox.getLeftX(),
                OperatorConstants.LEFT_X_DEADBAND);
        if (PoseTransformUtils.isRedAlliance()) {
            x *= -1;
        }
        double ySpeedTarget = m_xspeedLimiter.calculate(x);

        double y = MathUtil.applyDeadband(
                driverXbox.getLeftY(),
                OperatorConstants.LEFT_Y_DEADBAND);
        if (PoseTransformUtils.isRedAlliance()) {
            y *= -1;
        }
        double xSpeedTarget = m_yspeedLimiter.calculate(y);

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(xSpeedTarget, ySpeedTarget,
                rotation2d);

        if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond) {
            desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond, desiredSpeeds.omegaRadiansPerSecond);
        }

        swerve.driveFieldOriented(desiredSpeeds);
    }
}
