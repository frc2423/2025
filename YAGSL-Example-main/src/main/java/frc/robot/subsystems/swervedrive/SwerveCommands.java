package frc.robot.subsystems.swervedrive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

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
import swervelib.SwerveDrive;
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
        Transform2d offset = new Transform2d(distance, .178, Rotation2d.kPi);
        Pose2d targetPose = pose.plus(offset);
        return targetPose;
    }

    public Command autoAlign(Pose2d pose, double dist) {
        // Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);
        Pose2d targetPose = addScoringOffset(pose, dist);// .55

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                addScoringOffset(pose, dist + .5),
                addScoringOffset(pose, dist));
        PathConstraints constraints = new PathConstraints(
                .5, 1.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        PathPlannerPath path = new PathPlannerPath(
                waypoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));
        path.preventFlipping = true;
        Command pathFollower = AutoBuilder.followPath(path);
        // Since AutoBuilder is configured, we can use it to build pathfinding commands

        pathFollower.setName("Align to Pose");
        pathFollower.addRequirements(swerve);

        return pathFollower;
    }

    public Command stopMoving() {
        Command stopCommand = Commands.runOnce(() -> swerve.drive(new ChassisSpeeds()));
        stopCommand.addRequirements(swerve);
        return stopCommand;
    }

    public Command autoScoral(Pose2d pose, double setpoint) { // put in desired pose and elevator subsystem
        swerve.centerModulesCommand();
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                autoAlign(pose, 1.2),
                stopMoving(),
                elevatorSubsystem.goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                autoAlign(pose, .4),
                // new MoveForward(swerve, addScoringOffset(pose, .45).getTranslation(), 1),
                intakeCommands.intakeOut());

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
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));

        }).until(() -> (driverXbox.getRightX() > .1) || (driverXbox.getRightY() > .1));
        command.addRequirements(swerve);
        return command;
    }

    // public Command lookAtTarget(Pose2d targetAngle, Rotation2d offset) { // to
    // var command = Commands.sequence(
    // Commands.runOnce(currentAngleFilter::reset),
    // Commands.run(() -> {
    // Pose2d transformedPose = PoseTransformUtils.transformXRedPose(targetAngle);
    // specialAngle = swerve.getLookAngle(transformedPose).plus(offset);
    // swerve.actuallyLookAngle(specialAngle);
    // }, swerve).until(() -> {
    // double desiredAngle = normalizedAngle(specialAngle.getDegrees());
    // double currentAngle = currentAngleFilter
    // .calculate(normalizedAngle(swerve.getHeading().getDegrees()));
    // double angleDiff = getNormalizedAngleDiff(desiredAngle, currentAngle);
    // return angleDiff < 3;
    // }),
    // Commands.run(() -> {
    // Pose2d transformedPose = PoseTransformUtils.transformXRedPose(targetAngle);
    // specialAngle = swerve.getLookAngle(transformedPose).plus(offset);
    // swerve.actuallyLookAngle(specialAngle);
    // }, swerve).withTimeout(.5),
    // Commands.runOnce(() -> {
    // swerve.stop();
    // }));

    // command.setName("setLookAngle");
    // return command;
    // }

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
