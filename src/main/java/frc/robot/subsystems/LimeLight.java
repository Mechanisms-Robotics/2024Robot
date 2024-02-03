//package frc.robot.subsystems;
//
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//
//import java.util.Optional;
//
//public class LimeLight extends SubsystemBase {
//    private final String limelightName;
//    private final NetworkTable limelightTable;
//
//    static class AprilTagData {
//        public static final AprilTagData NOTHING = new AprilTagData(0.0, 0.0, 0.0);
//        public final double x;
//        public final double y;
//        public final double a;
//
//        public AprilTagData(double x, double y, double a) {
//            this.x = x;
//            this.y = y;
//            this.a = a;
//        }
//
//
//    }
//
//    public LimeLight(String name) {
//        this.limelightName = name;
//        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
//    }
//
//    @Override
//    public void periodic() {
//        AprilTagData aprilTagData = getAprilTagData().orElse(AprilTagData.NOTHING);
////        SmartDashboard.putNumber("tx", );
//    }
//
//    public Optional<AprilTagData> getAprilTagData() {
//        double x = limelightTable.getEntry("tx").getDouble(0.0);
//        double y = limelightTable.getEntry("ty").getDouble(0.0);
//        double a = limelightTable.getEntry("ta").getDouble(0.0);
//        return x == 0 && y == 0.0 && a == 0.0
//                ? Optional.of(new AprilTagData(x, y, a))
//                : Optional.empty();
//    }
//}
