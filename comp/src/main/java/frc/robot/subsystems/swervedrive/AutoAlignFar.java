package frc.robot.subsystems.swervedrive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignFar extends Command {
    private Pose2d pose;
    private boolean isRight;
    private double dist;
    private SwerveCommands swerveCommands;
    private SwerveSubsystem swerve;
    private boolean isAlgae = false;
    private Optional<Supplier<Pose2d>> climbPose = Optional.empty();

    private boolean reachedX = false;
    private boolean reachedY = false;
    private Optional<Integer> tagNumber = Optional.empty();
    private Timer timerY = new Timer();

    public AutoAlignFar(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.addRequirements(swerve);
    }

    public AutoAlignFar(SwerveSubsystem swerve, SwerveCommands swerveCommands, double dist, boolean isRight,
            Optional<Integer> tagNumber) {
        this.isRight = isRight;
        this.dist = dist;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.tagNumber = tagNumber;
        this.addRequirements(swerve);
    }

    public AutoAlignFar(SwerveSubsystem swerve, SwerveCommands swerveCommands, Supplier<Pose2d> climberPoseSupplier) {
        this.isAlgae = true;
        this.swerve = swerve;
        this.swerveCommands = swerveCommands;
        this.addRequirements(swerve);
        this.climbPose = Optional.of(climberPoseSupplier);
    }

    @Override
    public void initialize() {
        if (tagNumber.isPresent()) {
            pose = Vision.getTagPose(tagNumber.get());
        } else if (climbPose.isEmpty()) {
            pose = Vision.getTagPose(swerve.vision.findClosestTagID(swerve.getPose()));
        } else {
            pose = climbPose.get().get();
        }

        reachedX = false;
        reachedY = false;
        timerY.reset();
        timerY.start();
    }

    @Override
    public void execute() {
        Pose2d pose2d;

        if (!climbPose.isPresent()) {
            pose2d = isAlgae ? swerveCommands.addOffset(pose, dist, .1)
                    : swerveCommands.addScoringOffset(pose, dist, isRight);
        } else {
            pose2d = pose;
        }

        Pose2d robotPose = new Pose2d(swerve.getPose().getTranslation(), pose2d.getRotation());
        Translation2d translationDiff = pose2d.relativeTo(robotPose).getTranslation();

        double xDistance = Math.abs(translationDiff.getX());
        double yDistance = Math.abs(translationDiff.getY());

        if (xDistance < Units.inchesToMeters(2)) {
            reachedX = true;
        }
        if (yDistance < Units.inchesToMeters(2)) {
            timerY.start();
            reachedY = true;
        }
        if (timerY.get() >= 0.5 && yDistance > Units.inchesToMeters(2)) {
            reachedY = false;
            timerY.reset();
        }
        if (yDistance < Units.inchesToMeters(2)) {
            reachedY = true;
        }

        swerveCommands.actuallyMoveToFar(pose2d, !reachedX, !reachedY/* timerY.get() <= 0.5 */);
    }

    @Override
    public boolean isFinished() {
        return reachedX && reachedY;
    }
}