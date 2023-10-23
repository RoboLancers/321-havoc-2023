/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.robolancers321.Constants;

public class Vision extends SubsystemBase implements AutoCloseable {

  public static PhotonCamera camera = new PhotonCamera("photonvision");

  public boolean hasTargets = false;

  public PhotonPipelineResult result = camera.getLatestResult();

  public List<PhotonTrackedTarget> targets = result.targets;

  public PhotonTrackedTarget bestTarget = result.getBestTarget();

  public AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

  public PhotonPoseEstimator defaultPoseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
          camera,
          Constants.Vision.kCamOffset);



  public Vision() throws IOException {}

  @Override
  public void periodic() {

    // update periodically
    hasTargets = this.result.hasTargets();
    targets = this.result.getTargets();
    bestTarget = this.result.getBestTarget();
  }

  public void setPoseStrategy(PhotonPoseEstimator.PoseStrategy strategy) {
      this.defaultPoseEstimator.setPrimaryStrategy(strategy);
  }

  public int getTargetID() {
    return this.bestTarget.getFiducialId();
  }

  public Transform3d getTargetDistance() {
    return bestTarget.getBestCameraToTarget();
  }

  @Override
  public void close() {}
}
