package frc.robot.subsystems;

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
    private int aprilTagID = 8;
    private double yaw = 0;
    private double area = 0;

    public double getYaw() {
        return yaw;
    }

    public double getArea() {
        return area;
    }

    public boolean hasTarget() {
        return getArea() != 0;
    }

    @Override
    public void periodic() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().equals(Alliance.Blue)) aprilTagID = 8;
            else aprilTagID = 4;
        } else {
            System.out.println("The Alliance was no found");
        }

        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTarget = result.hasTargets();
        SmartDashboard.putBoolean("Has Target", hasTarget);
        if (hasTarget) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget target;
            target = targets.stream().filter(t -> t.getFiducialId() == aprilTagID).toList().get(0);
            if (target != null) {
                yaw = target.getYaw();
                area = target.getArea();
                return;
            }
        }
        yaw = 0;
        area = 0;
    }
}