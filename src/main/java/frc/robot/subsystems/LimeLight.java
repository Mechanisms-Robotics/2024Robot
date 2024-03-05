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

    public record LimeLightData(
            double yaw,
            double area,
            boolean hasTarget
    ) {}

    public LimeLightData getData() {
        return new LimeLightData(yaw, area, area!=0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[Lime Light] Target Yaw", yaw);
        SmartDashboard.putNumber("[Lime Light] Target Area", area);
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) aprilTagID = 7;
            else aprilTagID = 4;
        } else { System.out.println("The Alliance was no found"); }

        SmartDashboard.putNumber("[Lime Light] Target ID", aprilTagID);

        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTarget = result.hasTargets();
        SmartDashboard.putBoolean("[Lime Light] Has Target", hasTarget);
        if (hasTarget) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() == aprilTagID) {
                    yaw = target.getYaw();
                    area = target.getArea();
                    // turn on the LED if yaw is within 5 degrees
                    if (Math.abs(yaw) < 5) camera.setLED(VisionLEDMode.kOn);
                    else camera.setLED(VisionLEDMode.kOff);
                    return;
                }
            }
            // turn off the LED, it no work
        } else { camera.setLED(VisionLEDMode.kOff); }
        yaw = 0;
        area = 0;
    }
}
