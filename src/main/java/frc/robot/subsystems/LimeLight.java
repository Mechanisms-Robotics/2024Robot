package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("LimeLight");
    private int aprilTagID = 7;
    private double yaw = 0;
    private double area = 0;
    private Transform3d fieldToCamera = new Transform3d();

    /**
     * Data of the LimeLight
     *
     * @param yaw angle made by the center of the camera and the target
     * @param area % area that the target takes up on the camera, used for distance
     * @param hasTarget true if the camera sees an AprilTag otherwise false
     */

    public record LimeLightData(
            double yaw,
            double area,
            boolean hasTarget,
            Transform3d fieldToCamera
    ) {}

    /**
     * Returns object LimeLightData with all data of the LimeLight.
     * hasTarget is true if the area is not 0 (if the target is not showing on the camera.
     *
     * @return data of the LimeLight: yaw, area, and hasTarget
     */
    public LimeLightData getData() {
        return new LimeLightData(yaw, area, area!=0, fieldToCamera);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Lime Light] Target Yaw", yaw);
        SmartDashboard.putNumber("[Lime Light] Target Area", area);
        // If the Alliance is Blue, look for AprilTag 7, otherwise (when it is red) look for AprilTag 4
        // If there is no alliance specified, print error
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) aprilTagID = 7;
            else aprilTagID = 4;
        } else { System.out.println("The Alliance was no found"); }
        SmartDashboard.putNumber("[Lime Light] Target ID", aprilTagID);


        // get the data from the camera
        PhotonPipelineResult result = camera.getLatestResult();
        // if the april tag result is present, reset the fieldToCamera Transfrom3D
        if (result.getMultiTagResult().estimatedPose.isPresent)
            fieldToCamera = result.getMultiTagResult().estimatedPose.best;

        boolean hasTarget = result.hasTargets();
        SmartDashboard.putBoolean("[Lime Light] Has Target", hasTarget);
        // if any of the targets is the correct AprilTag, set the yaw and area
        // otherwise set them to 0
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == aprilTagID) {
                yaw = target.getYaw();
                area = target.getArea();
                return;
            }
        }
        yaw = 0;
        area = 0;
    }
}
