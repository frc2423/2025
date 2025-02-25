package frc.robot.subsystems.swervedrive;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.PoseTransformUtils;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.AngleUtils;
import frc.robot.Constants;
import frc.robot.NTHelper;
import frc.robot.Constants.OperatorConstants;

public class SwerveCommands {

    private SwerveSubsystem swerve;
    private IntakeCommands intakeCommands;

    PIDController translationPIDX = new PIDController(3.5, 0, .3);
    PIDController translationPIDY = new PIDController(3.5, 0, .3);

    private ElevatorSubsystem elevatorSubsystem;
    private XboxController driverXbox = new XboxController(0);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(7);
    private final String[] DEFAULT_ELEVATOR_LEVEL = { "off" };

    public SwerveCommands(SwerveSubsystem swerve, ElevatorSubsystem elevatorSubsystem,
            IntakeCommands intakeCommands) {
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeCommands = intakeCommands;
        NTHelper.setStringArray("/elevatorLevel", DEFAULT_ELEVATOR_LEVEL);
    }

    // The enum used as keys for selecting the command to run.
    private enum ElevatorLevel {
        T, L2, L3, L4
    }

    // An example selector method for the selectcommand. Returns the selector that
    // will select
    // which command to run. Can base this choice on logical conditions evaluated at
    // runtime.
    private ElevatorLevel selectElevatorLevel() {
        String dashboardElevatorLevel = NTHelper.getStringArray("/elevatorLevel", DEFAULT_ELEVATOR_LEVEL)[0];
        if (dashboardElevatorLevel.equals("L1")) {
            return ElevatorLevel.T;
        }
        if (dashboardElevatorLevel.equals("L2")) {
            return ElevatorLevel.L2;
        }
        if (dashboardElevatorLevel.equals("L3")) {
            return ElevatorLevel.L3;
        }
        if (dashboardElevatorLevel.equals("L4")) {
            return ElevatorLevel.L4;
        }

        if (driverXbox.getLeftTriggerAxis() < 0.5 && driverXbox.getRightTriggerAxis() < 0.5) {
            return ElevatorLevel.T;
        } else if (driverXbox.getLeftTriggerAxis() < 0.5 && driverXbox.getRightTriggerAxis() > 0.5) {
            return ElevatorLevel.L2;
        } else if (driverXbox.getLeftTriggerAxis() > 0.5 && driverXbox.getRightTriggerAxis() < 0.5) {
            return ElevatorLevel.L3;
        } else {
            return ElevatorLevel.L4;
        }
    }

    public Pose2d addScoringOffset(Pose2d pose, double distance, boolean isRight) {// robot POV
        double y = .178;
        double offsetY = (isRight ? y : -y) - Units.inchesToMeters(5);
        Transform2d offset = new Transform2d(distance, offsetY, Rotation2d.kPi);
        Pose2d targetPose = pose.plus(offset);
        return targetPose;
    }

    public Command stopMoving() {
        Command stopCommand = Commands.runOnce(() -> swerve.drive(new ChassisSpeeds()));
        stopCommand.addRequirements(swerve);
        return stopCommand;
    }

    public Command autoScoralClosest(boolean isRight) {
        Command autoScoreElevatorCommand = new SelectCommand<>(
                // Maps selector values to commands
                Map.ofEntries(
                        Map.entry(ElevatorLevel.T, elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.ZERO)),
                        Map.entry(ElevatorLevel.L2,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L2)),
                        Map.entry(ElevatorLevel.L3,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L3)),
                        Map.entry(ElevatorLevel.L4,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L4))),

                this::selectElevatorLevel);
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                new AutoAlignClosest(swerve, this, .8, isRight),
                stopMoving(),
                autoScoreElevatorCommand,
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                stopMoving(),
                new AutoAlignClosest(swerve, this, .32, isRight),
                stopMoving(),
                intakeCommands.intakeOut());

        command.setName("autoScoral");
        // command.addRequirements(swerve, elevatorSubsystem);

        return command;
    }

    public Command autoScoralClosest(double setpoint, boolean isRight) {
        Command autoScoreElevatorCommand = new SelectCommand<>(
                // Maps selector values to commands
                Map.ofEntries(
                        Map.entry(ElevatorLevel.T, elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.ZERO)),
                        Map.entry(ElevatorLevel.L2,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L2)),
                        Map.entry(ElevatorLevel.L3,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L3)),
                        Map.entry(ElevatorLevel.L4,
                                elevatorSubsystem.goToSetpoint(Constants.SetpointConstants.REEF_L4))),

                this::selectElevatorLevel);
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                new AutoAlignClosest(swerve, this, .8, isRight),
                stopMoving(),
                autoScoreElevatorCommand,
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                stopMoving(),
                new AutoAlignClosest(swerve, this, .4, isRight),
                stopMoving(),
                intakeCommands.intakeOut());

        command.setName("autoScoral");
        // command.addRequirements(swerve, elevatorSubsystem);

        return command;
    }

    public Command autoScoral(Pose2d pose, double setpoint, boolean isRight) { // put in desired pose and elevator
                                                                               // subsystem
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                autoAlign(pose, .8, isRight),
                stopMoving(),
                elevatorSubsystem.goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                autoAlign(pose, .32, isRight),
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
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle).plus(Rotation2d.k180deg));

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

    public Command autoAlign(Pose2d pose2d, double dist, boolean isRight) {
        Pose2d targetPose = addScoringOffset(pose2d, dist, isRight);// .55
        var command = Commands.run(() -> {
            actuallyMoveTo(targetPose);
        }).until(() -> {
            double xDistance = Math.abs(targetPose.getX() - swerve.getPose().getX());
            double yDistance = Math.abs(targetPose.getY() - swerve.getPose().getY());
            boolean isAngleClose = AngleUtils.areAnglesClose(targetPose.getRotation(), swerve.getPose().getRotation(),
                    Rotation2d.fromDegrees(5));
            return xDistance < 0.0508 &&
                    yDistance < 0.0508 &&
                    isAngleClose;
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

        double xDistance = Math.abs(pose2d.getX() - swerve.getPose().getX());
        double yDistance = Math.abs(pose2d.getY() - swerve.getPose().getY());

        if (xDistance > 0.0508) {
            x = Math.copySign(Math.max(.35, Math.abs(x)), x);
        }

        if (yDistance > 0.0508) {
            y = Math.copySign(Math.max(.35, Math.abs(y)), y);
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
