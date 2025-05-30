/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.vision;

 import static frc.robot.Constants.Vision.*;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.wpilibj.smartdashboard.Field2d;
 import frc.robot.NTHelper;
 import frc.robot.PoseTransformUtils;
 import frc.robot.Robot;
 
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.estimation.TargetModel;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.simulation.VisionTargetSim;
 import org.photonvision.targeting.PhotonPipelineResult;
 
 public class Vision {
     private final PhotonCamera aprilTagCamera;
     private final PhotonCamera noteCamera;
     private final PhotonPoseEstimator photonEstimator;
     private double lastEstTimestamp = 0;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;
 
     public Vision() {
         aprilTagCamera = new PhotonCamera(kCameraName);
         noteCamera = new PhotonCamera(knoteCameraName);
         noteCamera.setPipelineIndex(0);
 
         photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
                 
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the
             // field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this
             // simulated field.
             visionSim.addAprilTags(kTagLayout);
             // Pose2d(Translation2d(X: 3.72, Y: 5.41), Rotation2d(Rads: 1.19, Deg: 68.40))
             var targetPose = new Pose3d(3.72, 5.41, 1.25, new Rotation3d());
             var targetModel = new TargetModel(0.2);
             VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel, 40);
             // Add this vision target to the vision system simulation to make it visible
             visionSim.addVisionTargets(visionTarget);
             // Create simulated camera properties. These can be set to mimic your actual
             // camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values
             // with visible
             // targets.
             cameraSim = new PhotonCameraSim(aprilTagCamera, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
 
             cameraSim.enableDrawWireframe(true);
         }
     }
 
     public boolean isAprilTagCameraConnected() {
         return aprilTagCamera.isConnected();
     }
 
     public boolean isNoteCameraConnected() {
         return noteCamera.isConnected();
     }
 
     public PhotonPipelineResult getLatestResult() {
         var result = aprilTagCamera.getLatestResult();
         if (result.hasTargets()) {
             var target = result.getBestTarget();
             NTHelper.setDouble("/best target/yaw", target.getYaw());
 
             var pose3d = kTagLayout.getTagPose(target.getFiducialId());
             if (pose3d.isPresent()) {
                 double pitch = Rotation2d.fromDegrees(target.getPitch()).getRadians();
                 double yaw = Rotation2d.fromDegrees(target.getYaw()).getRadians();
                 Transform3d transform = new Transform3d(new Translation3d(), new Rotation3d(0, pitch, yaw));
                 Pose3d newPose = pose3d.get().plus(transform);
                 NTHelper.setDoubleArray("/best target/pose", NTHelper.getDoubleArrayPose3d(newPose));
             }
         } else {
             NTHelper.setDoubleArray("/best target/pose", new double[] { 0, 0, 100, 0, 0, 0, 0 });
         }
         return result;
     }
 
     public PhotonPipelineResult getLatestNoteResult() {
         return noteCamera.getLatestResult();
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be
      * empty. This should
      * only be called once per loop.
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
      *         timestamp, and targets
      *         used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         var visionEst = photonEstimator.update();
         double latestTimestamp = aprilTagCamera.getLatestResult().getTimestampSeconds();
         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
         if (Robot.isSimulation()) {
             visionEst.ifPresentOrElse(
                     est -> getSimDebugField()
                             .getObject("VisionEstimation")
                             .setPose(est.estimatedPose.toPose2d()),
                     () -> {
                         if (newResult)
                             getSimDebugField().getObject("VisionEstimation").setPoses();
                     });
         }
         if (newResult)
             lastEstTimestamp = latestTimestamp;
         return visionEst;
     }
 
     /**
      * The standard deviations of the estimated pose from
      * {@link #getEstimatedGlobalPose()}, for use
      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
      * SwerveDrivePoseEstimator}.
      * This should only be used when there are targets visible.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      */
     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
         var estStdDevs = kSingleTagStdDevs;
         var targets = getLatestResult().getTargets();
         int numTags = 0;
         double avgDist = 0;
         for (var tgt : targets) {
             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
             if (tagPose.isEmpty())
                 continue;
             numTags++;
             avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
         }
         if (numTags == 0)
             return estStdDevs;
         avgDist /= numTags;
         // Decrease std devs if multiple targets are visible
         if (numTags > 1)
             estStdDevs = kMultiTagStdDevs;
         // Increase std devs based on (average) distance
         if (numTags == 1 && avgDist > 4)
             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
         else
             estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
 
         return estStdDevs;
     }
 
     public PhotonCamera getAprilTagCamera() {
         return aprilTagCamera;
     }
 
     public PhotonCamera getNoteCamera() {
         return noteCamera;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation())
             visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation())
             return null;
         return visionSim.getDebugField();
     }
 }