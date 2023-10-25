/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.vision;

import static org.robolancers321.Constants.Vision.kMultiTagStdDevs;
import static org.robolancers321.Constants.Vision.kSingleTagStdDevs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robolancers321.Constants;
import org.robolancers321.Robot;

public class Vision implements AutoCloseable {

  private double lastEstTimestamp;

  public enum VisionPipeline {
    APRIL_TAG(0),
    REFLECTIVE(1);

    public final int pipelineIndex;

    VisionPipeline(int i) {
      this.pipelineIndex = i;
    }
  }

  public PhotonCamera camera = new PhotonCamera("photonvision");

  public boolean hasTargets = false;

  public boolean connected = false;

  public PhotonPipelineResult result = camera.getLatestResult();

  public List<PhotonTrackedTarget> targets = result.targets;

  public PhotonTrackedTarget bestTarget = result.getBestTarget();

  public AprilTagFieldLayout aprilTagFieldLayout;

  {
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public PhotonPoseEstimator defaultPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
          camera,
          Constants.Vision.kCamOffset);

  public SimVisionSystem simVision;

  public Vision() {
    this.defaultPoseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

    // Simulation

    if (Robot.isSimulation()) {

      // Configure these to match your PhotonVision Camera,
      // pipeline, and LED setup.
      double camDiagFOV = 170.0; // degrees - assume wide-angle camera
      double camPitch = Constants.Vision.kCamPitch; // degrees
      double camHeightOffGround = Constants.Vision.kCamOffset.getY(); // meters
      double maxLEDRange = 20; // meters
      int camResolutionWidth = 640; // pixels
      int camResolutionHeight = 480; // pixels
      double minTargetArea = 10; // square pixels

      this.simVision =
          new SimVisionSystem(
              "photonvision",
              camDiagFOV,
              new Transform3d(
                  new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
              maxLEDRange,
              camResolutionWidth,
              camResolutionHeight,
              minTargetArea);
    }
  }

  public boolean checkConnection() {

    String sdKey = "CAMERA CONNECTED";

    SmartDashboard.putBoolean(sdKey, this.camera.isConnected());

    return SmartDashboard.getBoolean(sdKey, false);
  }

  public void setPoseStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
    this.defaultPoseEstimator.setPrimaryStrategy(strategy);
  }

  public Optional<Matrix<N3, N3>> getCameraMat() {
    return this.camera.getCameraMatrix();
  }

  public Optional<Matrix<N5, N1>> getDistCoeffs() {
    return this.camera.getDistCoeffs();
  }

  public VisionPipeline getPipeline() {

    if (this.camera.getPipelineIndex() == 1) {
      return VisionPipeline.REFLECTIVE;
    }

    return VisionPipeline.APRIL_TAG;
  }

  public int getTargetID() {
    return this.bestTarget.getFiducialId();
  }

  public Transform3d getTargetDistance() {
    return bestTarget.getBestCameraToTarget();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

    return this.defaultPoseEstimator.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    this.defaultPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return this.defaultPoseEstimator.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose3d prevEstimatedRobotPose) {
    this.defaultPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return this.defaultPoseEstimator.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFromPrevious(Pose2d lastPose) {
    this.defaultPoseEstimator.setLastPose(lastPose);

    return this.defaultPoseEstimator.update();
  }

  public EstimatedRobotPose getEstimatedGlobalPoseFromPrevious(Pose3d lastPose) {
    this.defaultPoseEstimator.setLastPose(lastPose);

    return this.defaultPoseEstimator.update().get();
  }

  //  From Photon docs
  // https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = kSingleTagStdDevs;
    var targets = this.targets;
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = defaultPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  @Override
  public void close() {}
}
