package frc.robot.subsystems.swervedrive;

import java.lang.module.ModuleDescriptor.Builder;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    ProfiledPIDController alignFarPID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(10, 10));

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
        double offsetY = (isRight ? .178 : -.148) - Units.inchesToMeters(5);
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

    // public Command autoDescoreAlgae(double setpoint) {
    // var command = Commands.parallel(
    // elevatorSubsystem.descoreAlgae(setpoint),
    // new AutoAlign(swerve, this, 1, Optional.empty(), -.1),
    // new AutoAlign(swerve, this, .4, Optional.empty(), -.1));
    // command.setName("autoDescorAlgae");
    // return command;
    // }

    // public Command autoAlignAndDescoreAlgae(double setpoint) {
    // Command descoreCommand = elevatorSubsystem.descoreAlgae(setpoint);
    // Command autoAlignCommand1 = new AutoAlignFar(swerve, this, 1, true,
    // Optional.empty());
    // Command autoAlignCommand2 = new AutoAlignNear(swerve, this, .4, true,
    // Optional.empty());
    // var command = Commands.sequence(
    // autoAlignCommand1,
    // Commands.sequence(
    // descoreCommand,
    // Commands.waitUntil(() -> {
    // return elevatorSubsystem.isAtSetpoint() && armSubsystem.isAtSetpoint();
    // }),
    // autoAlignCommand2));
    // command.setName("autoDescorAlgaeButBetter");
    // return command;
    // }

    public Command autoAlignAndIntakeAlgae(double setpoint) {
        // Command dunkIt =
        // Commands.sequence(armSubsystem.goToSetpoint(Constants.ArmConstants.ALGAE_DUNK),
        // intakeCommands.intakeAlgae());
        Command autoAlignCommand1 = new AutoAlignFar(swerve, this, .6, Optional.empty()); // yo yo auto align
        Command autoAlignCommand2 = new AutoAlignNear(swerve, this, .4, Optional.empty());
        Command autoAlignCommand3 = new AutoAlignNear(swerve, this, .6, Optional.empty());

        var command = Commands.sequence(
                Commands.deadline(
                        Commands.sequence(
                                autoAlignCommand1,
                                Commands.waitUntil(() -> {
                                    return elevatorSubsystem.isAtSetpoint();
                                }),
                                autoAlignCommand2),
                        elevatorSubsystem.intakeAlgae(setpoint)),
                Commands.parallel(
                        armSubsystem.goToSetpoint(Constants.ArmConstants.ALGAE_HOLD),
                        Commands.sequence(
                                Commands.waitSeconds(.5),
                                autoAlignCommand3)));
        // intakeAlgaeCommand2));
        command.setName("autoIntakeAlgae");
        return command;
    }

    public Command autoScoral(Optional<Integer> tagNumber, Command elevatorLevelCommand, boolean isRight) {
        Command goScoreCommand = Commands.either(armSubsystem.goScoreL4(), armSubsystem.goScore(),
                () -> elevatorSubsystem.getSetpoint() > 50);
        Command autoAlignNearCommand = Commands.either(new AutoAlignNear(swerve, this, 0.51, isRight, tagNumber),
                new AutoAlignNear(swerve, this, 0.47, isRight, tagNumber),
                () -> elevatorSubsystem.getSetpoint() > 50).withTimeout(2);

        Command prepareElevator = Commands.sequence(
                Commands.waitUntil(() -> swerve.getPose().getTranslation().getDistance(
                        (PoseTransformUtils.isRedAlliance()) ? new Translation2d(13.055, 4.007)
                                : new Translation2d(4.507, 4.031)) < 2),
                elevatorLevelCommand,
                Commands.waitUntil(() -> {
                    return elevatorSubsystem.isAtSetpoint();
                }),
                goScoreCommand,
                Commands.waitUntil(() -> {
                    return armSubsystem.isAtSetpoint();
                }));

        var command = Commands.sequence(
                Commands.parallel(prepareElevator,
                        Commands.sequence(new AutoAlignFar(swerve, this, 0.6, isRight, tagNumber),
                                Commands.waitSeconds(0.3),
                                autoAlignNearCommand)),
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

    public double getDistanceBetweenPoses(Pose2d a, Pose2d b) {
        double y = a.getY() - b.getY();
        double x = a.getX() - b.getX();
        return Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));
    }

    public Rotation2d getLookAngle(Pose2d targetPose) {
        Pose2d currentPose = swerve.getPose();
        double distance = getDistanceBetweenPoses(currentPose, targetPose);
        if (distance < Units.inchesToMeters(8)) {
            return currentPose.getRotation();
        }
        double angleRads = Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
        return new Rotation2d(angleRads);
    }

    static Pose2d reefCenter = new Pose2d(new Translation2d(13.055, 4.007), Rotation2d.fromDegrees(0));

    public Command orbitReefCenter() {
        var command = Commands.run(() -> {
            Rotation2d angle = getLookAngle(reefCenter);
            actuallyLookAngleButMove(angle);
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
        if (!PoseTransformUtils.isRedAlliance()) {
            x *= -1;
        }
        double ySpeedTarget = m_xspeedLimiter.calculate(x);

        double y = MathUtil.applyDeadband(
                driverXbox.getLeftY(),
                OperatorConstants.LEFT_Y_DEADBAND);
        if (!PoseTransformUtils.isRedAlliance()) {
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
        } else {
            // x = MathUtil.interpolate(0.5, 0.7, (xDistance - 0.5) / (0.7 - 0.5));
            x = .5;

        }

        if (!enableY) {
            y = 0;
        } else {
            // y = MathUtil.interpolate(0.5, 0.7, (yDistance - 0.5) / (0.7 - 0.5));
            y = .5;

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

    public void actuallyMoveToFar(Pose2d pose2d, boolean enableX, boolean enableY) {

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
        } else {
            x = MathUtil.interpolate(0.5, 1, (xDistance - 0.25) / (1.5 - 0.25));

        }

        // else if (xDistance > .7) {}
        // x = .6;
        // } else if (xDistance > .4) {
        // x = .6;
        // } else if (xDistance > .2) {
        // x = .55;
        // } else if (xDistance > .03) {
        // x = .55;
        // } else if (xDistance > .02) {
        // x = .55;
        // } else if (xDistance > .015) {
        // x = .5;
        // } else {
        // x = 0;
        // }

        if (!enableY) {
            y = 0;
        } else {
            y = MathUtil.interpolate(0.45, 0.9, (yDistance - 0.176) / (2 - 0.176));
        }

        // } else if (yDistance > .7) {
        // y = .6;
        // } else if (yDistance > .4) {
        // y = .6;
        // } else if (yDistance > .2) {
        // y = .55;
        // } else if (yDistance > .03) {
        // y = .55;
        // } else if (yDistance > .02) {
        // y = .55;
        // } else if (yDistance > .015) {
        // y = .5;
        // } else {
        // y = 0;
        // }

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
