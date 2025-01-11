package frc.robot.subsystems.vision;

import java.util.Optional;

import javax.print.attribute.standard.Media;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Vision;

public class VisionSubsystem extends SubsystemBase {
    private Vision visionInterface = new Vision();
    public double getLatestId = 0;
    private Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
    private PhotonPipelineResult aprilTagResult = visionInterface.getLatestResult();
    private PhotonPipelineResult noteResult = visionInterface.getLatestNoteResult();
    private MedianFilter noteFilter = new MedianFilter(5);
    private double noteAverage = 0;
    private double notePitch = 0;
    private double noteYaw = 0;


    @Override
    public void periodic() {
        estimatedPose = visionInterface.getEstimatedGlobalPose();
        aprilTagResult = visionInterface.getLatestResult();
        noteResult = visionInterface.getLatestNoteResult();
        noteAverage = noteFilter.calculate(seesRawNote() ? 1 : 0);
        if (noteResult.hasTargets()) {
            notePitch = noteResult.getBestTarget().getPitch();
            noteYaw = noteResult.getBestTarget().getYaw();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        return estimatedPose;
    }

    public Optional<Matrix<N3, N1>> getStandardDeviations() {
        Optional<EstimatedRobotPose> pose = getEstimatedRobotPose();
        if (pose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(visionInterface.getEstimationStdDevs(pose.get().estimatedPose.toPose2d()));
    }

    public double getTimestampSeconds() {
        return aprilTagResult.getTimestampSeconds();

    }

    public void simulationPeriodic(Pose2d pose) {
        visionInterface.simulationPeriodic(pose);
    }

    public Optional<Transform3d> getLatestResult() {
        if (!aprilTagResult.hasTargets()) {
            return null;
        }
        PhotonTrackedTarget getLatestResult = aprilTagResult.getBestTarget();
        if (getLatestResult != null) {
            getLatestId = getLatestResult.getFiducialId();
            // setLEDColor();
            return Optional.ofNullable(getLatestResult.getBestCameraToTarget());
        }
        return null;
    }

    public boolean seesAprilTag() {
        if (getLatestResult() != null) {
            return true;
        }
        return false;
    }

    public double getOffset() {
        if (!noteResult.hasTargets()) {
            return 0; //
        }
        double x = getX(notePitch);
        double y = getY(noteYaw);

        double angle = Math.tan(x / y);
        return angle;
    }

    public boolean seesRawNote() {
        return noteResult.hasTargets();
    }

    public boolean seesNote() {
        return noteAverage > .5;
    }

    public double getNoteYaw() {
        if (!seesNote()) {
            return 0;
        }
        return noteYaw;
    }

    public boolean isAlignedNote() {
        if (!seesNote()) {
            return false;
        }
        return Math.abs(noteYaw) < 4;
    }

    public double yawAfterAligned(){
        if (!seesNote()) {
            return 0;
        }
        if(Math.abs(noteYaw) < .5){
            return 0;
        } else {
            return -getNoteYaw() * .075;
        }
    }

    private double getX(double camx) {
        double x = (.163 * Math.pow(camx, 2)) + (1.298 * camx) + 28.7;
        return x;
    }

    private double getY(double camy) {
        double y = (.0376 * Math.pow(camy, 2)) + 4.2598;
        return y;
    }

    public Rotation2d getTurn() {
        // gets how much the robot needs to turn to get the note in the center
        return new Rotation2d(getOffset());
    }

    // public Pose2d getNotePose(Pose2d robotPose){
    // var result = noteVision.getLatestNoteResult();
    // if (result != null) {
    // //return 0; //
    // }
    // double x = robotPose.getX() + getX(result.getBestTarget().getPitch());
    // double y = robotPose.getY() + getY(result.getBestTarget().getYaw());

    // }

    public void updateNoteV() {
        // NTHelper.setBoolean("noteVision/connected", visionInterface.isAprilTagCameraConnected());
        // if (noteResult.hasTargets()) {
        //     NTHelper.setDouble("noteVision/x", noteResult.getBestTarget().getPitch());
        //     NTHelper.setDouble("noteVision/y", noteResult.getBestTarget().getYaw());
        // }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addBooleanProperty("has targets", () -> aprilTagResult.hasTargets(), null);
        builder.addBooleanProperty("is connected", () -> visionInterface.isAprilTagCameraConnected(), null);
        builder.addIntegerProperty("pipeline index", () -> visionInterface.getAprilTagCamera().getPipelineIndex(),
                null);
    }
}