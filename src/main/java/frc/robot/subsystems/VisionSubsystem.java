package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(VisionConstants.TAG_LAYOUT, VisionConstants.ROBOT_TO_CAMERA);

    private Matrix<N3, N1> curStdDevs;
    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;

    public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);
            
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);

            cameraSim = new PhotonCameraSim(camera, cameraProp);
            visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
            cameraSim.enableDrawWireframe(true);
        }
    }

        /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();  

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            visionEstimate = poseEstimator.estimateCoprocMultiTagPose(result);

            if (visionEstimate.isEmpty()) {
                visionEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(visionEstimate, result.getTargets());
        }

        visionEstimate.ifPresent(estimate -> {
            swerveSubsystem.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, curStdDevs);
        });
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(swerveSubsystem.getSimulationPose());
    }
}
