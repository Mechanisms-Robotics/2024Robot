package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class LimeLight extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("LimeLight");
    private int aprilTagID = 7;
    private double yaw = 0;
    private double area = 0;

    /**
     * Returns the yaw of the angle made between the camera and the target.
     * This serves to make sure that the target is centered on the camera for aiming at an april tag.
     *
     * @return the yaw of the angle made between the camera and the target
     */
    public double getYaw() {
        return yaw;
    }

    /**
     * Returns the area that the target takes on the limelight.
     * This is used for determininig the distance from the april tags.
     *
     * @return the area that the target takes on the limelight
     */
    public double getArea() {
        return area;
    }

    /**
     * Returns whether a target has been found
     *
     * @return whether a target has been found
     */
    public boolean hasTarget() {
        return getArea() != 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Target Yaw", yaw);
        SmartDashboard.putNumber("Target Area", area);
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) aprilTagID = 7;
            else aprilTagID = 4;
        } else {
            System.out.println("The Alliance was no found");
        }
        SmartDashboard.putNumber("[Lime Light] april tag target ID", aprilTagID);

        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTarget = result.hasTargets();
        SmartDashboard.putBoolean("[Lime Light] Has Target", hasTarget);
        if (hasTarget) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() == aprilTagID) {
                    yaw = target.getYaw();
                    area = target.getArea();
                    if (Math.abs(yaw) < 5) camera.setLED(VisionLEDMode.kOn);
                    else camera.setLED(VisionLEDMode.kOff);
                    return;
                }
            }
        } else {
            camera.setLED(VisionLEDMode.kOff);
        }
        yaw = 0;
        area = 0;
    }
}
