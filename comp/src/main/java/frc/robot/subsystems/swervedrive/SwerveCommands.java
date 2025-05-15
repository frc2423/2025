package frc.robot.subsystems.swervedrive;

import java.lang.module.ModuleDescriptor.Builder;
import java.time.OffsetTime;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.CustomParamConfiguration;

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
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorLevelPicker;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeCommands;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.AngleUtils;
import frc.robot.Constants;
import frc.robot.NTHelper;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveCommands {
    SendableChooser<String> climbPositionChooser = new SendableChooser<>();

    private SwerveSubsystem swerve;
    private ArmSubsystem armSubsystem;
    private IntakeCommands intakeCommands;
    private IntakeSubsystem intakesubsystem;
    private ClimberSubsystem climberSubsystem;
    private ElevatorLevelPicker elevatorLevelPicker;

    private ElevatorSubsystem elevatorSubsystem;
    private XboxController driverXbox = new XboxController(0);
    private final String[] DEFAULT_ELEVATOR_LEVEL = { "off" };

    ProfiledPIDController alignFarPID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(10, 10));

    public SwerveCommands(SwerveSubsystem swerve, ElevatorSubsystem elevatorSubsystem,
            IntakeCommands intakeCommands, ArmSubsystem armSubsystem, IntakeSubsystem intakesubsystem,
            ClimberSubsystem climberSubsystem) {
        this.intakesubsystem = intakesubsystem;
        this.elevatorLevelPicker = new ElevatorLevelPicker(elevatorSubsystem, swerve);
        this.swerve = swerve;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeCommands = intakeCommands;
        this.armSubsystem = armSubsystem;
        this.climberSubsystem = climberSubsystem;
        NTHelper.setStringArray("/elevatorLevel", DEFAULT_ELEVATOR_LEVEL);
        SmartDashboard.putData("climbChooser", climbPositionChooser);

        climbPositionChooser.addOption("Blue left (wall)", "Blue left (wall)");
        climbPositionChooser.addOption("Blue middle", "Blue middle");
        climbPositionChooser.addOption("Blue right (reef)", "Blue right (reef)");
        climbPositionChooser.addOption("Red left (wall)", "Red left (wall)");
        climbPositionChooser.addOption("Red middle", "Red middle");
        climbPositionChooser.addOption("Red right (reef)", "Red right (reef)");
        climbPositionChooser.setDefaultOption("Blue left (wall)", "Blue left (wall)");
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

    public Command lookAtNearestHPTag() {
        var command = Commands.run(() -> {
            int tag = swerve.vision.findClosestHPSTagID(swerve.getPose());
            int angle = swerve.vision.hpIDToAngle(tag);
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));
            // .plus(Rotation2d.k180deg));
        }).until(() -> (driverXbox.getRightX() > .1) || (driverXbox.getRightY() > .1));
        command.addRequirements(swerve);
        return command;
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

    public Pose2d getClimbPose2d(double offset) {
        switch (climbPositionChooser.getSelected()) {
            case "Blue left (wall)":
                return new Pose2d(7.121 - offset, 7.280, Rotation2d.fromDegrees(-90));
            case "Blue middle":
                return new Pose2d(7.121 - offset, 6.165, Rotation2d.fromDegrees(-90));
            case "Blue right (reef)":
                return new Pose2d(7.121 - offset, 5.075, Rotation2d.fromDegrees(-90));
            case "Red left (wall)":
                return new Pose2d(10.441 + offset, 3, Rotation2d.fromDegrees(90));
            case "Red middle":
                return new Pose2d(10.441 + offset, 1.885, Rotation2d.fromDegrees(90));
            case "Red right (reef)":
                return new Pose2d(10.441 + offset, 0.806, Rotation2d.fromDegrees(90));
            default:
                return new Pose2d(7.121 - offset, 7.280, Rotation2d.fromDegrees(-90));
        }
    }

    public Pose2d getClimbPose2d() {
        return getClimbPose2d(0);
    }

    public Pose2d getCloserClimbPose2d() {
        return getClimbPose2d(.5);
    }

    public Command autoAlignClimb() {
        Command command = Commands.sequence(
                new AutoAlignFar(swerve, this, this::getCloserClimbPose2d),
                Commands.parallel(new AutoAlignNear(swerve, this, this::getClimbPose2d), climberSubsystem.deClimb()),
                new MoveForward(swerve, Units.inchesToMeters(9), -.35),
                Commands.waitSeconds(2),
                Commands.either(climberSubsystem.climb(), Commands.none(), climberSubsystem::limitSwitch));
        command.setName("autoAlignClimb");
        return command;
    }

    public Pose2d getAutoIntakePose() {
        if (PoseTransformUtils.isRedAlliance()) {
            if (Vision.getAprilTagPose(1).getTranslation().getDistance(swerve.getPose().getTranslation()) >= Vision
                    .getAprilTagPose(2).getTranslation().getDistance(swerve.getPose().getTranslation())) {
                return new Pose2d(16.040, 7.280, Vision.getAprilTagPose(2).getRotation()); // go to 2
            } else {
                return new Pose2d(16.040, 0.698, Vision.getAprilTagPose(1).getRotation()); // go to 1
            }
        } else {
            if (Vision.getAprilTagPose(13).getTranslation().getDistance(swerve.getPose().getTranslation()) >= Vision
                    .getAprilTagPose(12).getTranslation().getDistance(swerve.getPose().getTranslation())) {
                return new Pose2d(1.570, 0.680, Vision.getAprilTagPose(12).getRotation()); // go to 12
            } else {
                return new Pose2d(1.570, 7.376, Vision.getAprilTagPose(13).getRotation()); // go to 13
            }
        }
    }

    public Command autoAlignAndIntakeHP() {
        Command command = Commands.parallel(
                Commands.sequence(new AutoAlignFar(swerve, this, this::getAutoIntakePose),
                        new AutoAlignNear(swerve, this, this::getAutoIntakePose)),
                elevatorSubsystem.goDownAndIntake());
        command.setName("autoAlignHP");
        return command;
    }

    public Command autoScoral(Optional<Integer> tagNumber, Command elevatorLevelCommand, boolean isRight) {
        Command goScoreCommand = Commands.either(armSubsystem.goScoreL4(), armSubsystem.goScore(),
                () -> elevatorSubsystem.getSetpoint() > 50);
        Command autoAlignNearCommand = Commands.either(new AutoAlignNear(swerve, this, 0.51, isRight, tagNumber),
                new AutoAlignNear(swerve, this, 0.47, isRight, tagNumber),
                () -> elevatorSubsystem.getSetpoint() > 50).withTimeout(2);

        Command prepareElevator = Commands.sequence(
                intakeCommands.in(),
                Commands.waitUntil(() -> swerve.getPose().getTranslation().getDistance(
                        (PoseTransformUtils.isRedAlliance()) ? new Translation2d(13.055, 4.007)
                                : new Translation2d(4.507, 4.031)) < 3),
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
                intakeCommands.intakeJustOutRun().withTimeout(.5), elevatorLevelPicker.setScoredLevel(),
                intakeCommands.intakeOut());

        command.setName("autoScoralClosest");

        return command;
    }

    public Command autoHPIntake(Optional<Boolean> isRight) {
        Command autoAlignHPCommand = new AutoAlignHP(swerve, this, isRight);
        return autoAlignHPCommand;
    }

    // public Pose2d autoIntakePose(){
    // if (PoseTransformUtils.isRedAlliance()) {
    // if
    // (Vision.getAprilTagPose(1).getTranslation().getDistance(swerve.getPose().getTranslation())
    // >= Vision
    // .getAprilTagPose(2).getTranslation().getDistance(swerve.getPose().getTranslation()))
    // {
    // return new Pose2d()
    // } else {
    // pose = new
    // }
    // } else {
    // if
    // (Vision.getAprilTagPose(13).getTranslation().getDistance(swerve.getPose().getTranslation())
    // >= Vision
    // .getAprilTagPose(12).getTranslation().getDistance(swerve.getPose().getTranslation()))
    // {
    // pose =
    // } else {
    // pose =
    // }
    // }

    // return pose;
    // }

    // public Command autoIntakeCoral() {
    // boolean isRed = PoseTransformUtils.isRedAlliance();
    // Commands.either(Commands.print("command 1"), Commands.print("Command 2"), ()
    // -> PoseTransformUtils.isRedAlliance());
    // Optional<Boolean> isRight = Optional.of(right);

    // var command = Commands.sequence(
    // new AutoAlignFar(autoIntakePose()),
    // new AutoAlignNear());
    // return command;
    // }

    public Command autoScoral(Optional<Integer> tagNumber, double setpoint, boolean isRight) {
        Command command = Commands.run(() -> {
            swerve.drive(new ChassisSpeeds(0, 0, 0));
        }).withTimeout(.2);
        command.addRequirements(swerve);
        return Commands.either(
                command,
                autoScoral(tagNumber, elevatorSubsystem.goToSetpoint(setpoint), isRight),
                () -> intakesubsystem.isOut());
        // return autoScoral(tagNumber, elevatorSubsystem.goToSetpoint(setpoint),
        // isRight);

    }

    public Command autoScoralClosest(boolean isRight) {
        return autoScoral(Optional.empty(), elevatorLevelPicker.getElevatorLevelCommand(), isRight);
    }

    public Command autoScoralClosestAuto() {
        Command scoreLeft = autoScoral(Optional.empty(), elevatorLevelPicker.getElevatorLevelCommandAuto(),
                true);
        Command scoreRight = autoScoral(Optional.empty(), elevatorLevelPicker.getElevatorLevelCommandAuto(),
                false);

        Command whichSide = Commands.either(scoreLeft, scoreRight, () -> elevatorLevelPicker.isRightOpen());
        return whichSide;
        // return Commands.sequence(whichSide, elevatorLevelPicker.setScoredLevel());
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

    static Pose2d REDreefCenter = new Pose2d(new Translation2d(13.055, 4.007), Rotation2d.fromDegrees(0));
    static Pose2d BLUEreefCenter = new Pose2d(new Translation2d(4.495, 4.019), Rotation2d.fromDegrees(0));

    private boolean isRightStickBeingUsed() {
        return Math.abs(driverXbox.getRightX()) > .1 || Math.abs(driverXbox.getRightY()) > .1;
    }

    public Command orbitReefCenter() {
        var command = Commands.run(() -> {

            int tag = swerve.vision.findClosestHPSTagID(swerve.getPose());
            double dist = swerve.vision.getDistanceFromAprilTag(tag);
            if (PoseTransformUtils.isRedAlliance()) {
                if (dist >= 1.5) {
                    Rotation2d angle = getLookAngle(REDreefCenter);
                    actuallyLookAngleButMove(angle);

                } else {
                    int hpAngle = swerve.vision.hpIDToAngle(tag);
                    actuallyLookAngleButMove(Rotation2d.fromDegrees(hpAngle));
                }
            } else {
                if (dist >= 3) {
                    Rotation2d angle = getLookAngle(BLUEreefCenter);
                    actuallyLookAngleButMove(angle);

                } else {
                    int hpAngle = swerve.vision.hpIDToAngle(tag);
                    actuallyLookAngleButMove(Rotation2d.fromDegrees(hpAngle));
                }

            }

        }).until(() -> (isRightStickBeingUsed()));
        command.addRequirements(swerve);
        return command;
    }

    public Command lookAtAngle(double angle) {
        var command = Commands.run(() -> {
            actuallyLookAngleButMove(Rotation2d.fromDegrees(angle));
        }).until(() -> (isRightStickBeingUsed()));
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
        double ySpeedTarget = swerve.m_xspeedLimiter.calculate(x);

        double y = MathUtil.applyDeadband(
                driverXbox.getLeftY(),
                OperatorConstants.LEFT_Y_DEADBAND);
        if (!PoseTransformUtils.isRedAlliance()) {
            y *= -1;
        }

        double xSpeedTarget = swerve.m_yspeedLimiter.calculate(y);

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
