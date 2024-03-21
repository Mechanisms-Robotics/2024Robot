package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;

public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("LimeLight");
    // TODO: get the transform 3d of the camera to the robot
    private final Transform3d cameraToRobot = new Transform3d(0, 0, 0, new Rotation3d());
    /** Creates the AprilTag data holder for the subwoofer */
    private final Tag subTag = new Tag(7, 4);
    /** The transform of the field to the camera */
    private Transform3d fieldToCamera = new Transform3d();
    private ArrayList<Pose3d> robotPositions = new ArrayList<>();
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


    /**
     * Data of the LimeLight
     *
     * @param robotPositions ArrayList of Pose3ds calculated by each detected april tag
     * @param fieldToCamera Transform3d of the field to the camera based on multiple april tags
     * @param aprilTagFieldLayout layout of all april tags on the field
     */

    public record LimeLightData(
            Tag subTag,
            ArrayList<Pose3d> robotPositions,
            Transform3d fieldToCamera,
            AprilTagFieldLayout aprilTagFieldLayout
    ) {}

    public class Tag {
        int aprilTagID;
        double yaw;
        double area;
        public Tag(int blueID, int redID, double yaw, double area) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) aprilTagID = blueID;
            else if (DriverStation.getAlliance().get() == Alliance.Red) aprilTagID = redID;
            else throw new RuntimeException("The alliance was not found and AprilTag ID culd no be set cuh");
            this.yaw = yaw;
            this.area = area;
        }
        public Tag(int blueID, int redID) {
            this(blueID, redID, 0, 0);
        }
        public record data(
                int aprilTagID,
                double yaw,
                double area,
                boolean hasTarget,
                boolean aimed
        ) {}
        public data getData() {
            return new data(aprilTagID, yaw, area, area!=0, MathUtil.isNear(0, yaw, 5));
        }
    }

    /**
     * Returns object LimeLightData with all data of the LimeLight.
     * hasTarget is true if the area is not 0 (if the target is not showing on the camera.
     *
     * @return data of the LimeLight: yaw, area, and hasTarget
     */
    public LimeLightData getData() {
        return new LimeLightData(subTag, robotPositions, fieldToCamera, aprilTagFieldLayout);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Lime Light] [Sub] Yaw", subTag.yaw);
        SmartDashboard.putNumber("[Lime Light] [Sub] Area", subTag.area);
        SmartDashboard.putNumber("[Lime Light] [Sub]", subTag.aprilTagID);

        // get the data from the camera
        PhotonPipelineResult result = camera.getLatestResult();
        // if the april tag result is present, reset the fieldToCamera Transfrom3D
        if (result.getMultiTagResult().estimatedPose.isPresent)
            fieldToCamera = result.getMultiTagResult().estimatedPose.best;

        boolean hasTarget = result.hasTargets();
        SmartDashboard.putBoolean("[Lime Light] Has Target", hasTarget);

        robotPositions.clear();
        // if any of the targets is the correct AprilTag, set the yaw and area
        // otherwise set them to 0
        for (PhotonTrackedTarget target : result.getTargets()) {
            // get the position of the robot based on the target
            if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
                        cameraToRobot);

                robotPositions.add(robotPose);
            }
            if (target.getFiducialId() == subTag.aprilTagID) {
                subTag.yaw = target.getYaw();
                subTag.area = target.getArea();
                return;
            }
        }
        subTag.yaw = 0;
        subTag.area = 0;
    }
}
