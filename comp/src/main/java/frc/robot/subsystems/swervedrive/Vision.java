package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.NTHelper;
import frc.robot.Robot;
import frc.robot.subsystems.QuackNav;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken
 * from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025ReefscapeAndyMark);
  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link Vision#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose
   *          Current pose supplier, should reference
   *          {@link SwerveDrive#getPose()}
   * @param field
   *          Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  public void logCameras() {
    for (Cameras cam : Cameras.values()) {
      cam.log();
    }
  }

  public static Pose2d getTagPose(int id) {
    return fieldLayout.getTagPose(id).get().toPose2d();
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag
   *          The ID of the AprilTag.
   * @param robotOffset
   *          The offset {@link Transform2d} of the robot to apply to the pose for
   *          the robot to position
   *          itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  public static Pose2d getAprilTagPose(int aprilTag) {
    return getAprilTagPose(aprilTag, new Transform2d());
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive
   *          {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive, QuackNav quackNav) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        NTHelper.setDouble("/swerveSubsystem/vision/poseX", pose.estimatedPose.getX());
        NTHelper.setDouble("/swerveSubsystem/vision/poseX", pose.estimatedPose.getY());
        NTHelper.setDouble("/swerveSubsystem/vision/poseX",
            pose.estimatedPose.getRotation().toRotation2d().getDegrees());

        if (!quackNav.isQuestMode()) {
          swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
              pose.timestampSeconds,
              camera.curStdDevs);
        } else {
          Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
              0.02, // Trust down to 2cm in X direction
              0.02, // Trust down to 2cm in Y direction
              0.035 // Trust down to 2 degrees rotational
          );
          // Get timestamp from the QuestNav instance
          double timestamp = quackNav.getTimestamp();

          // Convert FPGA timestamp to CTRE's time domain using Phoenix 6 utility
          double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);
          swerveDrive.addVisionMeasurement(quackNav.getPose(), ctreTimestamp,
              QUESTNAV_STD_DEVS);
        }

        quackNav.updateQuestPose(swerveDrive.getPose(), camera.curStdDevs);

        var stdDev = camera.curStdDevs;
        NTHelper.setDouble("/swerveSubsystem/vision/stdDevX", stdDev.get(0, 0));
        NTHelper.setDouble("/swerveSubsystem/vision/stdDevY", stdDev.get(1, 0));
        NTHelper.setDouble("/swerveSubsystem/vision/stdDevAngle", stdDev.get(2, 0));
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    NTHelper.setBoolean("/swerveSubsystem/vision/poseIsGood", filterPose(poseEst));
    if (!filterPose(poseEst)) {
      return Optional.empty();
    }
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  /**
   * Filter pose via the ambiguity and find best estimate between all of the
   * camera's throwing out distances more than
   * 10m for a short amount of time.
   *
   * @param pose
   *          Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  @Deprecated(since = "2024", forRemoval = true)
  private boolean filterPose(Optional<EstimatedRobotPose> pose) {
    if (pose.isPresent()) {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed) {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) {
          bestTargetAmbiguity = ambiguity;
        }
      }

      NTHelper.setDouble("/swerveSubsystem/vision/filter/bestTargetAmbiguity", bestTargetAmbiguity);
      NTHelper.setDouble("/swerveSubsystem/vision/filter/x", pose.get().estimatedPose.getX());
      NTHelper.setDouble("/swerveSubsystem/vision/filter/y", pose.get().estimatedPose.getY());
      NTHelper.setDouble("/swerveSubsystem/vision/filter/z", pose.get().estimatedPose.getZ());

      // ambiguity to high dont use estimate
      if (bestTargetAmbiguity > maximumAmbiguity) {
        return false;
      }

      if (pose.get().estimatedPose.getX() < 0 || pose.get().estimatedPose.getX() > fieldLayout.getFieldLength()) {
        return false;
      }
      if (pose.get().estimatedPose.getY() < 0 || pose.get().estimatedPose.getY() > fieldLayout.getFieldWidth()) {
        return false;
      }
      if (Math.abs(pose.get().estimatedPose.getZ()) > 0.32) {
        return false;
      }

      // est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1) {
        longDistangePoseEstimationCount++;

        // if it calculates that were 10 meter away for more than 10 times in a row its
        // probably right
        if (longDistangePoseEstimationCount < 10) {
          return false;
        }
      } else {
        longDistangePoseEstimationCount = 0;
      }
      return true;
    }
    return false;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id
   *          AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id
   *          AprilTag ID
   * @param camera
   *          Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  public Integer findClosestTagID(Pose2d currentPose) {
    int[] AprilTagIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
    List<Pose2d> poseList = new ArrayList<Pose2d>();
    Map<Pose2d, Integer> tagMap = new HashMap<Pose2d, Integer>();
    for (int tag : AprilTagIDs) {
      poseList.add(fieldLayout.getTagPose(tag).get().toPose2d());
      tagMap.put(fieldLayout.getTagPose(tag).get().toPose2d(), tag);
    }

    return tagMap.get(currentPose.nearest(poseList));

  }

  public Integer findClosestHPSTagID(Pose2d currentPose) {
    int[] AprilTagIDs = { 1, 2, 12, 13 };
    List<Pose2d> poseList = new ArrayList<Pose2d>();
    Map<Pose2d, Integer> tagMap = new HashMap<Pose2d, Integer>();
    for (int tag : AprilTagIDs) {
      poseList.add(fieldLayout.getTagPose(tag).get().toPose2d());
      tagMap.put(fieldLayout.getTagPose(tag).get().toPose2d(), tag);
    }

    return tagMap.get(currentPose.nearest(poseList));

  }

  public int iDtoAngle(int tag) {
    int[] AprilTagIDs = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    Map<Integer, Integer> tagMap = new HashMap<Integer, Integer>();

    tagMap.put(6, -60);
    tagMap.put(7, 0);
    tagMap.put(8, 60);
    tagMap.put(9, 120);
    tagMap.put(10, 180);
    tagMap.put(11, -120);
    tagMap.put(17, 60);
    tagMap.put(18, 0);
    tagMap.put(19, -60);
    tagMap.put(20, -120);
    tagMap.put(21, 180);
    tagMap.put(22, 120);

    return tagMap.get(tag);

  }

  public int hpIDToAngle(int tag) {
    // int[] AprilTagIDs = { 1, 2, 12, 13 };
    Map<Integer, Integer> tagMap = new HashMap<Integer, Integer>();
    tagMap.put(1, 120);
    tagMap.put(2, 240);
    tagMap.put(12, 60);
    tagMap.put(13, -60);
    return tagMap.get(tag);
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running
   * photon vision on localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      // try
      // {
      // Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      // } catch (IOException | URISyntaxException e)
      // {
      // e.printStackTrace();
      // }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /*
   * Check whether the front camera sees an April Tag
   * 
   * @return True when an april tag is discovered.
   */
  public boolean seesFrontAprilTag() {
    return Cameras.FRONT_RIGHT_CAM.hasTarget() || Cameras.FRONT_LEFT_CAM.hasTarget();
  }

  /**
   * Camera Enum to select each camera
   * 
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-drive-kinematics
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#rotation-conventions
   * 
   */
  enum Cameras {
    FRONT_RIGHT_CAM("right_cam",
        new Rotation3d(0, Math.toRadians(-20), Math.toRadians(0)),
        new Translation3d(Units.inchesToMeters(10.5), // center to front
            Units.inchesToMeters(-5),
            Units.inchesToMeters(6)), // front floor
        VecBuilder.fill(2, 2, 8), VecBuilder.fill(0.5, 0.5, 1)),

    FRONT_LEFT_CAM("left_cam",
        new Rotation3d(0, Math.toRadians(-20), Math.toRadians(-45)),
        new Translation3d(Units.inchesToMeters(5.707),
            Units.inchesToMeters(9.793),
            Units.inchesToMeters(7.75)),
        VecBuilder.fill(2, 2, 8), VecBuilder.fill(0.5, 0.5, 1));

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    private boolean isTargetPresent = false;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name
     *          Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation
     *          {@link Rotation3d} of the camera.
     * @param robotToCamTranslation
     *          {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs
     *          Single AprilTag standard deviations of estimated poses from the
     *          camera.
     * @param multiTagStdDevsMatrix
     *          Multi AprilTag standard deviations of estimated poses from the
     *          camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    public double getEstPoseX() {
      if (estimatedRobotPose.isEmpty()) {
        return -100000000;
      }
      return estimatedRobotPose.get().estimatedPose.getX();
    }

    public double getEstPoseY() {
      if (estimatedRobotPose.isEmpty()) {
        return -100000000;
      }
      return estimatedRobotPose.get().estimatedPose.getY();
    }

    public double getEstPoseRot() {
      if (estimatedRobotPose.isEmpty()) {
        return -100000000;
      }
      return Math.toDegrees(estimatedRobotPose.get().estimatedPose.getRotation().getAngle());
    }

    public double getEstPoseZ() {
      if (estimatedRobotPose.isEmpty()) {
        return -100000000;
      }
      return estimatedRobotPose.get().estimatedPose.getZ();
    }

    public void log() {
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/stdDevX", getCurrentStdDevsX());
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/stdDevY", getCurrentStdDevsY());
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/stdDevRot", getCurrentStdDevsRot());
      NTHelper.setBoolean("/visionDebug/" + camera.getName() + "/camerasConnected", camera.isConnected());
      NTHelper.setBoolean("/visionDebug/" + camera.getName() + "/seesTag/", hasTarget());
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/estimatedPoseX", getEstPoseX());
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/estimatedPoseY", getEstPoseY());
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/estimatedPoseRot", getEstPoseRot());
      // NTHelper.setDouble("/visionDebug" + camera.getName() + "/height",);
      NTHelper.setDouble("/visionDebug/" + camera.getName() + "/poseAmbiguity", getPoseAmbiguityFromBestTarget());
      // NTHelper.getBoolean("/visionDebug/" + camera.getName() + "rejectingTag",
      // isRejecting);
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim
     *          {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    public double getPoseAmbiguityFromBestTarget() {
      if (resultsList.isEmpty()) {
        return -100000;
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;

      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }

      return bestResult.getBestTarget().getPoseAmbiguity();
    }

    /*
     * Check whether there was a recently discovered target
     * in the latest photon vision pipelne.
     * 
     * @return True when the target exists. False otherwise.
     */
    public boolean hasTarget() {
      return isTargetPresent;
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is
     *         there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms.
     * Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
          (currentTimestamp - lastReadTimestamp) >= debounceTime) {
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty()) {
          // Set the boolean when target is present
          isTargetPresent = resultsList.get(0).hasTargets();

          updateEstimatedGlobalPose();
        }
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      resultsList.clear();
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose
     *          The estimated pose to guess standard deviations for.
     * @param targets
     *          All targets in this camera frame
     */

    private Double getCurrentStdDevsX() {
      return curStdDevs.get(0, 0);
    }

    private Double getCurrentStdDevsY() {
      return curStdDevs.get(1, 0);
    }

    private Double getCurrentStdDevsRot() {
      return curStdDevs.get(2, 0);
    }

    // private Boolean isRejecting = null;

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;
        return;
      }

      // Pose present. Start running Heuristic
      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
          estStdDevs = multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 8) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          // isRejecting = true;
        } else if (avgDist <= 1) {
          estStdDevs = estStdDevs.times(.5); // number subject to change
          // isRejecting = false;
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          // isRejecting = false;
        }
        curStdDevs = estStdDevs;
        // herp
      }

    }

  }

}
