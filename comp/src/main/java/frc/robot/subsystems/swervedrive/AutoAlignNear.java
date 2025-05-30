package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.AngleUtils;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorLevelPicker;

public class AutoAlignNear extends Command {
    private Pose2d pose;
    private boolean isRight;
    private boolean isAlgae;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private ElevatorLevelPicker elevatorLevelPicker;
    private boolean reachedY = false;
    private boolean reachedX = false;
    private Optional<Integer> tagNumber = Optional.empty();
    private Optional<Supplier<Pose2d>> climbPose = Optional.empty();

    private final static int FILTER_SIZE = 10;
    MedianFilter targetAngleFilter = new MedianFilter(FILTER_SIZE);
    MedianFilter swerveAngleFilter = new MedianFilter(FILTER_SIZE);

    public AutoAlignNear(RobotContainer container, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.elevatorLevelPicker = container.elevatorLevelPicker;
        this.addRequirements(swerve);
    }

    public AutoAlignNear(RobotContainer container, double dist, boolean isRight,
            Optional<Integer> tagNumber) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.elevatorLevelPicker = container.elevatorLevelPicker;
        this.tagNumber = tagNumber;
        this.addRequirements(swerve);
    }

    public AutoAlignNear(RobotContainer container, double dist,
            Optional<Integer> tagNumber) {
        this.isAlgae = true;
        this.dist = dist;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.elevatorLevelPicker = container.elevatorLevelPicker;
        this.tagNumber = tagNumber;
        this.addRequirements(swerve);
    }

    public AutoAlignNear(RobotContainer container, Supplier<Pose2d> climberPoseSupplier) {
        this.isAlgae = true;
        this.swerve = container.drivebase;
        this.swerveCommands = container.swerveCommands;
        this.elevatorLevelPicker = container.elevatorLevelPicker;
        this.addRequirements(swerve);
        this.climbPose = Optional.of(climberPoseSupplier);
    }

    @Override
    public void initialize() {
        reachedY = false;
        reachedX = false;
        if (tagNumber.isPresent()) {
            pose = Vision.getTagPose(tagNumber.get());
        } else if (climbPose.isEmpty()) {
            pose = elevatorLevelPicker.getNearestOpenReefPose();
        } else {
            pose = climbPose.get().get();
        }

        targetAngleFilter.reset();
        swerveAngleFilter.reset();
        for (double i = 0; i < FILTER_SIZE; i++) {
            targetAngleFilter.calculate(100000);
            swerveAngleFilter.calculate(100000);
        }
    }

    @Override
    public void execute() {
        Pose2d pose2d;

        if (climbPose.isPresent())
            pose2d = climbPose.get().get();
        else if (isAlgae)
            pose2d = swerveCommands.addOffset(pose, dist, .1);
        else
            pose2d = swerveCommands.addScoringOffset(pose, dist, isRight);

        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());

        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        if (yDistance < Units.inchesToMeters(.45)) {
            reachedY = true;
        }
        if (xDistance < Units.inchesToMeters(.45)) {
            reachedX = true;
        }

        swerveCommands.actuallyMoveTo(pose2d, reachedY && !reachedX, !reachedY);
    }

    @Override
    public boolean isFinished() {
        Pose2d targetPose = climbPose.isEmpty() ? swerveCommands.addScoringOffset(pose, dist, isRight)
                : climbPose.get().get();// .55

        double averageTargetPose = targetAngleFilter.calculate(targetPose.getRotation().getRadians());
        double averageSwervePose = swerveAngleFilter.calculate(swerve.getPose().getRotation().getRadians());
        boolean isAngleClose = AngleUtils.areAnglesClose(new Rotation2d(averageTargetPose),
                new Rotation2d(averageSwervePose),
                Rotation2d.fromDegrees(3));

        return reachedY && reachedX && isAngleClose;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds());
    }
}
