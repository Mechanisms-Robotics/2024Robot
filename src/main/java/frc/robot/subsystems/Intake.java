package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.mechlib.hardware.TalonFX;
import com.mechlib.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX rightPivotMotor = new TalonFX(20);
   private final TalonFX leftPivotMotor = new TalonFX(21);

   private final double stowedPosition = 10000.0;
   private final double deployedPosition = 43500.00;
   private final double feedForward = -0.07;
   private boolean isZeroed = false;

   public Intake() {
       rightPivotMotor.brakeMode();
       leftPivotMotor.brakeMode();
       leftPivotMotor.setInverted(false);

       rightPivotMotor.follow(leftPivotMotor, true);

       leftPivotMotor.setKP(0.00002);
       leftPivotMotor.setKF(feedForward);
   }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "Intake Position",
                getPosition()
        );

        if (isZeroed) {
            leftPivotMotor.periodicPIDF(getPosition());
        }
    }

    public double getPosition() {
       return leftPivotMotor.getPosition();
    }

    public void pivotMotor() {
       rightPivotMotor.setPercent(0.05);
   }




   public void stopMotor() {
       rightPivotMotor.setPercent(0.0);
   }

   public void zero() {
       if (isZeroed) {return;}
       leftPivotMotor.zero();
       isZeroed = true;
    }


    /*
     * concept code for intake deployment and retraction in a single command
     * values will need to be tuned to properly work
     * while loop is likely sketchy option
     *

   public void setIntake() {
       if (isDeployed == true) {
           if (rightPivotMotor.getPosition() > stowedPosition) {
               rightPivotMotor.setPercent(-.5);
           }
           isDeployed = false;
       }
       else if (isDeployed == false) {
           if (rightPivotMotor.getPosition() < deployedPosition) {
               rightPivotMotor.setPercent(.5);
           }
           isDeployed = true;
           rightPivotMotor.setPercent(feedForward);
       }
   }

     */

   public void deploy() {
       leftPivotMotor.setSetpoint(deployedPosition);

   }

   public void retract() {
       leftPivotMotor.setSetpoint(stowedPosition);
   }


}