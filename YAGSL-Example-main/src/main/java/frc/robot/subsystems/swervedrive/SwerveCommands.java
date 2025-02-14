package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.PoseTransformUtils;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.Constants.OperatorConstants;

public class SwerveCommands {

    private SwerveSubsystem swerve;
    private IntakeCommands intakeCommands;

    PIDController translationPIDX = new PIDController(3.5, .1, 0);
    PIDController translationPIDY = new PIDController(3.5, .1, 0);

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

    public Command stopMoving() {
        Command stopCommand = Commands.runOnce(() -> swerve.drive(new ChassisSpeeds()));
        stopCommand.addRequirements(swerve);
        return stopCommand;
    }

    public Command autoScoral(Pose2d pose, double setpoint) { // put in desired pose and elevator subsystem
        swerve.centerModulesCommand();
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                autoAlign(pose, .8),
                stopMoving(),
                elevatorSubsystem.goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                autoAlign(pose, .32),
                stopMoving(),
                intakeCommands.intakeOut());

        command.setName("autoScoral");
        // command.addRequirements(swerve, elevatorSubsystem);

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

    public Command lookAtAngle(double angle) {
        var command = Commands.run(() -> {
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));
        }).until(() -> (driverXbox.getRightX() > .1) || (driverXbox.getRightY() > .1));
        command.addRequirements(swerve);
        return command;
    }

    public Command autoAlign(Pose2d pose2d, double dist) {
        Pose2d targetPose = addScoringOffset(pose2d, dist);// .55
        var command = Commands.run(() -> {
            actuallyMoveTo(targetPose);
        }).until(() -> {
            return targetPose.getTranslation().getDistance(swerve.getPose().getTranslation()) < 0.05;
        });
        command.addRequirements(swerve);
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

    public void actuallyMoveTo(Pose2d pose2d) { // here
        final double maxRadsPerSecond = 5;

        double x = translationPIDX.calculate(
                swerve.getPose().getX(), pose2d.getX());
        if (PoseTransformUtils.isRedAlliance()) {
            // x *= -1;
        }
        double y = translationPIDY.calculate(
                swerve.getPose().getY(), pose2d.getY());
        if (PoseTransformUtils.isRedAlliance()) {
            // y *= -1;
        }

        if (Math.abs(x) > 0.6) {
            x = Math.copySign(0.6, x);
        }
        if (Math.abs(y) > 0.6) {
            y = Math.copySign(0.6, y);
        }

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(x, y,
                pose2d.getRotation());

        if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond) {
            desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond, desiredSpeeds.omegaRadiansPerSecond);
        }

        swerve.driveFieldOriented(desiredSpeeds);
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
}
