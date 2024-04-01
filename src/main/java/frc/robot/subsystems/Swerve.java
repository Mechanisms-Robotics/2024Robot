package frc.robot.subsystems;

import com.mechlib.hardware.BrushlessMotorControllerType;
import com.mechlib.swerve.HeadingControllerConfiguration;
import com.mechlib.swerve.SwerveDrive;
import com.mechlib.swerve.SwerveModuleConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Swerve extends SwerveDrive {
    private static final int FL_STEER_MOTOR_ID = 5;
    private static final int FL_STEER_ENCODER_ID = 5;
    private static final double FL_STEER_OFFSET = 0.387 + 0.5;
    private static final boolean FL_STEER_INVERTED = true;
    private static final int FL_DRIVE_MOTOR_ID = 1;

    private static final int FR_STEER_MOTOR_ID = 6;
    private static final int FR_STEER_ENCODER_ID = 6;
    private static final double FR_STEER_OFFSET = -0.049 + 0.5;
    private static final boolean FR_STEER_INVERTED = true;
    private static final int FR_DRIVE_MOTOR_ID = 2;

    private static final int BR_STEER_MOTOR_ID = 7;
    private static final int BR_STEER_ENCODER_ID = 7;
    private static final double BR_STEER_OFFSET = 0.0844 + 0.5;
    private static final boolean BR_STEER_INVERTED = true;
    private static final int BR_DRIVE_MOTOR_ID = 3;

    private static final int BL_STEER_MOTOR_ID = 8;
    private static final int BL_STEER_ENCODER_ID = 8;
    private static final double BL_STEER_OFFSET = -0.382 + 0.5;
    private static final boolean BL_STEER_INVERTED = true;
    private static final int BL_DRIVE_MOTOR_ID = 4;

    private static final double DRIVE_GEAR_RATIO = 6.12;
    private static final double WHEEL_DIAMETER = 0.1012;

    // Module locations of the modules (the center of the wheel), all .24 meters from the center
    private static final Translation2d FL_MODULE_LOCATION = new Translation2d(
            .24,
            .24
    );

    private static final Translation2d FR_MODULE_LOCATION = new Translation2d(
            .24,
            -.24
    );

    private static final Translation2d BR_MODULE_LOCATION = new Translation2d(
            -.24,
            -.24
    );

    private static final Translation2d BL_MODULE_LOCATION = new Translation2d(
            -.24,
            .24
    );

    private static final int GYRO_ID = 9;

    /**
     * Stops the serve from driving or turning
     */
    public void stop() {
        drive(0, 0, 0);
    }

    public Swerve() {
        super(
                FL_STEER_MOTOR_ID,
                FL_STEER_ENCODER_ID,
                FL_DRIVE_MOTOR_ID,
                FL_STEER_OFFSET,
                true,

                FR_STEER_MOTOR_ID,
                FR_STEER_ENCODER_ID,
                FR_DRIVE_MOTOR_ID,
                FR_STEER_OFFSET,
                true,

                BR_STEER_MOTOR_ID,
                BR_STEER_ENCODER_ID,
                BR_DRIVE_MOTOR_ID,
                BR_STEER_OFFSET,
                true,

                BL_STEER_MOTOR_ID,
                BL_STEER_ENCODER_ID,
                BL_DRIVE_MOTOR_ID,
                BL_STEER_OFFSET,
                true,

                BrushlessMotorControllerType.TalonFX,
                BrushlessMotorControllerType.SparkMax,
                SwerveModuleConfiguration.DEFAULT,

                FL_MODULE_LOCATION,
                FR_MODULE_LOCATION,
                BR_MODULE_LOCATION,
                BL_MODULE_LOCATION,

                GYRO_ID,
                HeadingControllerConfiguration.GOBLIN
        );
    }
}
