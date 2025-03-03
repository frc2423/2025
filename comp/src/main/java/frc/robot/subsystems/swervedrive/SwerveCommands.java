package frc.robot.subsystems.swervedrive;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.PoseTransformUtils;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.AngleUtils;
import frc.robot.Constants;
import frc.robot.NTHelper;
import frc.robot.Constants.OperatorConstants;

public class SwerveCommands {

    private SwerveSubsystem swerve;
    private ArmSubsystem armSubsystem;
    private IntakeCommands intakeCommands;

    private ElevatorSubsystem elevatorSubsystem;
    private XboxController driverXbox = new XboxController(0);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(7);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(7);
    private final String[] DEFAULT_ELEVATOR_LEVEL = { "off" };

    public SwerveCommands(SwerveSubsystem swerve, ElevatorSubsystem elevatorSubsystem,
            IntakeCommands intakeCommands, ArmSubsystem armSubsystem) {
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeCommands = intakeCommands;
        this.armSubsystem = armSubsystem;
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
        return addOffset(pose, distance, offsetY);
    }

    public Pose2d addOffset(Pose2d pose, double distance, double offsetY) {// robot POV
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
                new AutoAlignClosest(swerve, this, 0.8, isRight),
                stopMoving(),
                Commands.parallel(
                        Commands.sequence(
                                autoScoreElevatorCommand,
                                Commands.waitUntil(() -> {
                                    return elevatorSubsystem.isAtSetpoint();
                                }),
                                armSubsystem.goScore()),
                        Commands.sequence(
                                swerve.centerModulesCommand().withTimeout(.3),
                                new AutoAlignClosest(swerve, this, 0.4, isRight),
                                stopMoving())),
                // stopMoving(),
                // new AutoAlignClosest(swerve, this, .4, isRight),
                // stopMoving(),
                intakeCommands.intakeOut());

        command.setName("autoScoral");
        // command.addRequirements(swerve, elevatorSubsystem);

        return command;
    }

    public Command autoScoral(Pose2d pose, double setpoint, boolean isRight) { // put in desired pose and elevator
                                                                               // subsystem
        var command = Commands.sequence(
                swerve.centerModulesCommand().withTimeout(.5),
                autoScoringAlign(pose, .8, isRight),
                stopMoving(),
                elevatorSubsystem.goToSetpoint(setpoint),
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                autoScoringAlign(pose, .32, isRight),
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

    public Command autoAlign(Pose2d pose2d, double dist, double offsetY) {
        Pose2d targetPose = addOffset(pose2d, dist, offsetY);
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

    public Command autoScoringAlign(Pose2d pose2d, double dist, boolean isRight) {
        double y = .178;
        double offsetY = (isRight ? y : -y) - Units.inchesToMeters(5);
        return autoAlign(pose2d, dist, offsetY);
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

        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());
        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        double xSign = Math.copySign(1, translationDiff.getX());
        double ySign = Math.copySign(1, translationDiff.getY());

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        double distance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));

        double x = 0;
        double y = 0;

        if (xDistance > .7) {
            x = Math.copySign(.6, xSign);
        } else if (xDistance > .3) {
            x = Math.copySign(.6, xSign);
        } else if (xDistance > .05) {
            x = Math.copySign(.4, xSign);
        } else if (xDistance > .03) {
            x = Math.copySign(.35, xSign);
        } else {
            x = 0;
        }

        if (yDistance > .7) {
            y = Math.copySign(.6, ySign);
        } else if (yDistance > .3) {
            y = Math.copySign(.6, ySign);
        } else if (yDistance > .05) {
            y = Math.copySign(.4, ySign);
        } else if (yDistance > .03) {
            y = Math.copySign(.35, ySign);
        } else {
            y = 0;
        }

        //
        // if (yDistance > .5) {
        // x *= .75;
        // }

        // if (distance < .2) {
        // if (xDistance > Units.inchesToMeters(2)) {
        // y = 0;
        // } else {
        // x = 0;
        // }
        // if (yDistance < Units.inchesToMeters(2)) {
        // y = 0;
        // }
        // }

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(x, y,
                pose2d.getRotation());

        final double maxRadsPerSecond = 5;

        if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond) {
            desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond,
                    desiredSpeeds.omegaRadiansPerSecond);
        }

        swerve.drive(desiredSpeeds);
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
