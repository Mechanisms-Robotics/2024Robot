//package frc.robot.commands.auto;
//
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//
//public class TopLeave {
//    public static final double MAX_VEL = 2.0; // m/s
//    public static final double MAX_ACCEL = 2.0; // m/s^2
//
//    private static final PathPlannerTrajectory MOBILITY_AUTO_WALL =
//            PathPlanner.loadPath("MobilityAutoWall", MAX_VEL, MAX_ACCEL);
//
//    public static CommandBase mobilityAutoWall(AutoBuilder autoBuilder) {
//        return autoBuilder.followPath(MOBILITY_AUTO_WALL, true);
//    }
//}