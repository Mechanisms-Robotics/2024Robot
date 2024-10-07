package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** LimeLight subsystem, contains and updates the data of the limelight */
public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("LimeLight");
    private int aprilTagID = 7;
    private double yaw = 0;
    private double area = 0;

    /**
     * Data of the LimeLight
     *
     * @param yaw angle made by the center of the camera and the target
     * @param area % area that the target takes up on the camera, used for distance
     * @param hasTarget true if the camera sees an AprilTag otherwise false
     * @param aimed true if the camera is pointed at the target
     */
    public record LimeLightData(
            double yaw,
            double area,
            boolean hasTarget,
            boolean aimed
    ) {}

    /**
     * Returns object LimeLightData with all data of the LimeLight.
     * hasTarget is true if the area is not 0 (if the target is not showing on the camera.
     *
     * @return data of the LimeLight: yaw, area, and hasTarget
     */
    public LimeLightData getData() {
        return new LimeLightData(yaw, area, area!=0,
                                 MathUtil.isNear(0, yaw, 5) && area!=0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Lime Light] Target Yaw", yaw);
        SmartDashboard.putNumber("[Lime Light] Target Area", area);
        SmartDashboard.putBoolean("[Lime Light] aimed", MathUtil.isNear(0, yaw, 5) && area!=0);
        // If the Alliance is Blue, look for AprilTag 7, otherwise (when it is red) look for AprilTag 4
        // If there is no alliance specified, print error
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) aprilTagID = 7;
            else aprilTagID = 4;
        } else if (Robot.isReal()) { System.out.println("The Alliance was no found"); }
        SmartDashboard.putNumber("[Lime Light] Target ID", aprilTagID);

        PhotonPipelineResult result = camera.getLatestResult();
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
