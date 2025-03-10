package frc.robot.subsystems.swervedrive;

import java.util.Map;
import java.util.Optional;

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
import frc.robot.subsystems.Arm.ArmSubsystem;
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

    public Vision getVisionFromSwerve() {
        return swerve.vision;
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

    public double getScoringOffset(boolean isRight) {
        double y = .178;
        double offsetY = (isRight ? y : -y) - Units.inchesToMeters(5);
        return offsetY;
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

    public Command getElevatorLevelCommand() {
        return new SelectCommand<>(
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
    }

    public Command autoDescorAlgae(double setpoint) {
        var command = Commands.parallel(

                elevatorSubsystem.descoreAlgae(setpoint),
                new AutoAlign(swerve, this, .4, Optional.empty(), -.1));
        command.setName("autoDescorAlgae");
        return command;
    }

    public Command autoScoral(Optional<Integer> tagNumber, Command elevatorLevelCommand, boolean isRight) {
        Command goScoreCommand = Commands.either(armSubsystem.goScoreL4(), armSubsystem.goScore(),
                () -> elevatorSubsystem.getSetpoint() > 50);
        Command autoAlignNearCommand = Commands.either(new AutoAlignNear(swerve, this, 0.47, isRight, tagNumber),
                new AutoAlignNear(swerve, this, 0.43, isRight, tagNumber),
                () -> elevatorSubsystem.getSetpoint() > 50).withTimeout(2);
        Command autoAlignNearCommand2 = Commands.either(new AutoAlignNear(swerve, this, 0.47, isRight, tagNumber),
                new AutoAlignNear(swerve, this, 0.43, isRight, tagNumber),
                () -> elevatorSubsystem.getSetpoint() > 50).withTimeout(2);

        var command = Commands.sequence(
                new AutoAlign(swerve, this, 1, isRight, tagNumber),
                stopMoving(),
                Commands.parallel(
                        Commands.sequence(
                                elevatorLevelCommand,
                                Commands.waitUntil(() -> {
                                    return elevatorSubsystem.isAtSetpoint();
                                }),
                                goScoreCommand),
                        Commands.sequence(
                                Commands.waitSeconds(.6),
                                autoAlignNearCommand,
                                autoAlignNearCommand2,
                                stopMoving())),
                intakeCommands.intakeOut());

        command.setName("autoScoralClosest");

        return command;
    }

    public Command autoScoral(Optional<Integer> tagNumber, double setpoint, boolean isRight) {
        return autoScoral(tagNumber, elevatorSubsystem.goToSetpoint(setpoint), isRight);
    }

    public Command autoScoralClosest(boolean isRight) {
        return autoScoral(Optional.empty(), getElevatorLevelCommand(), isRight);
    }

    public Command autoScoralClosest(double setpoint, boolean isRight) {
        return autoScoral(Optional.empty(), elevatorSubsystem.goToSetpoint(setpoint), isRight);
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

    public void actuallyMoveTo(Pose2d pose2d) {
        actuallyMoveTo(pose2d, true, true);
    }

    public void actuallyMoveTo(Pose2d pose2d, boolean enableX, boolean enableY) {

        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());
        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        boolean isAngleClose = AngleUtils.areAnglesClose(pose2d.getRotation(),
                swerve.getPose().getRotation(),
                Rotation2d.fromDegrees(1));

        double xSign = Math.copySign(1, translationDiff.getX());
        double ySign = Math.copySign(1, translationDiff.getY());

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        double x = 0;
        double y = 0;

        if (!enableX) {
            x = 0;
        } else if (xDistance > .7) {
            x = .8;
        } else if (xDistance > .4) {
            x = .6;
        } else if (xDistance > .2) {
            x = .55;
        } else if (xDistance > .03) {
            x = .53;
        } else if (xDistance > .02) {
            x = .47;
        } else if (xDistance > .015) {
            x = .43;
        } else {
            x = 0;
        }

        if (!enableY) {
            y = 0;
        } else if (yDistance > .7) {
            y = .8;
        } else if (yDistance > .4) {
            y = .6;
        } else if (yDistance > .2) {
            y = .47;
        } else if (yDistance > .03) {
            y = .42;
        } else if (yDistance > .02) {
            y = .35;
        } else if (yDistance > .015) {
            y = .3;
        } else {
            y = 0;
        }

        x *= xSign;
        y *= ySign;

        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(x, y,
                pose2d.getRotation());

        final double maxRadsPerSecond = 5;

        if (isAngleClose) {
            // desiredSpeeds.omegaRadiansPerSecond = 0;
        } else if (Math.abs(desiredSpeeds.omegaRadiansPerSecond) > maxRadsPerSecond) {
            desiredSpeeds.omegaRadiansPerSecond = Math.copySign(maxRadsPerSecond,
                    desiredSpeeds.omegaRadiansPerSecond);
        }

        swerve.drive(desiredSpeeds);
    }
}
