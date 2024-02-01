package frc.robot.subsystems;

import com.mechlib.hardware.BrushlessMotorControllerType;
import com.mechlib.swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;

public class Swerve extends SwerveDrive {

    private static final int FL_STEER_MOTOR_ID = 5;
    private static final int FL_STEER_ENCODER_ID = 5;
    private static final double FL_STEER_OFFSET = 85.605;
    private static final boolean FL_STEER_INVERTED = true;
    private static final int FL_DRIVE_MOTOR_ID = 1;

    private static final int FR_STEER_MOTOR_ID = 6;
    private static final int FR_STEER_ENCODER_ID = 6;
    private static final double FR_STEER_OFFSET = -21.182;
    private static final boolean FR_STEER_INVERTED = true;
    private static final int FR_DRIVE_MOTOR_ID = 2;

    private static final int BR_STEER_MOTOR_ID = 7;
    private static final int BR_STEER_ENCODER_ID = 7;
    private static final double BR_STEER_OFFSET = 30.762;
    private static final boolean BR_STEER_INVERTED = true;
    private static final int BR_DRIVE_MOTOR_ID = 3;

    private static final int BL_STEER_MOTOR_ID = 8;
    private static final int BL_STEER_ENCODER_ID = 8;
    private static final double BL_STEER_OFFSET = 172.793;
    private static final boolean BL_STEER_INVERTED = true;
    private static final int BL_DRIVE_MOTOR_ID = 4;

    private static final double DRIVE_GEAR_RATIO = 6.12;
    private static final double WHEEL_DIAMETER = 0.1012;

    private static final Translation2d FL_MODULE_LOCATION = new Translation2d(
            0.384175,
            0.231775
    );

    private static final Translation2d FR_MODULE_LOCATION = new Translation2d(
            0.384175,
            -0.231775
    );

    private static final Translation2d BR_MODULE_LOCATION = new Translation2d(
            -0.384175,
            -0.231775
    );

    private static final Translation2d BL_MODULE_LOCATION = new Translation2d(
            -0.384175,
            0.231775
    );

    private static final int GYRO_ID = 0;

    public Swerve() {
        super(
                FL_STEER_MOTOR_ID,
                FL_STEER_ENCODER_ID,
                FL_STEER_OFFSET,
                FL_STEER_INVERTED,
                FL_DRIVE_MOTOR_ID,

                FR_STEER_MOTOR_ID,
                FR_STEER_ENCODER_ID,
                FR_STEER_OFFSET,
                FR_STEER_INVERTED,
                FR_DRIVE_MOTOR_ID,

                BR_STEER_MOTOR_ID,
                BR_STEER_ENCODER_ID,
                BR_STEER_OFFSET,
                BR_STEER_INVERTED,
                BR_DRIVE_MOTOR_ID,

                BL_STEER_MOTOR_ID,
                BL_STEER_ENCODER_ID,
                BL_STEER_OFFSET,
                BL_STEER_INVERTED,
                BL_DRIVE_MOTOR_ID,

                BrushlessMotorControllerType.TalonFX,
                BrushlessMotorControllerType.SparkMax,

                DRIVE_GEAR_RATIO,
                WHEEL_DIAMETER,

                FL_MODULE_LOCATION,
                FR_MODULE_LOCATION,
                BR_MODULE_LOCATION,
                BL_MODULE_LOCATION,

                GYRO_ID
        );
    }
}
