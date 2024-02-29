package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the arm and the wrist as one subsystem because the states are the same.
 */
public class ArmWrist extends SubsystemBase {
    private static final Arm arm = new Arm();
    private static final Wrist wrist = new Wrist();
    private boolean safe = false;

    public void disable() {

    }

    private enum State {
        Stowed,
        Intaking,
        Amping,
        ShootingSubwoofer,
        ShootingPodium
    }
    private State state;
    private boolean lowMode = false;

    /**
     * Set the arm and wrist to the stow position.
     * Updates the position to stow and does not run when in safe mode (safe=true)
     */
    public void stow() {
        if (safe) return; // do not run if in safety mode
        // if the state is not already stowed set the state to stow and set the arm and wrist position to stow
        if (state != State.Stowed) {
            arm.stow();
            wrist.stow();
            state = State.Stowed;
        }
    }

    /**
     * Set the arm and wrist to intake position.
     * Sets the state to intaking
     */
    public void intake() {
        // if the state is not already intaking set the state to intaking and set the arm and wrist position to intaking
        if (state != State.Intaking) {
            arm.intake();
            wrist.intake();
            state = State.Intaking;
        }
    }

    /**
     * Set the arm and wrist to the subwoofer position: low or high
     *
     * @param lowMode true if shooting low and false if shooting high
     */
    public void subwooferShoot(boolean lowMode) {
        this.lowMode = lowMode;
        if (safe) return; // do not run if in safe mode
        /* if the state is not already in shooting subwoofer, shoot in low or high mode (specified in parameter lowMode)
        and set the state to ShootingSubwoofer */
        if (state != State.ShootingSubwoofer) {
            // if in low mode, set the arm and wrist position to shooting low
            if (lowMode) {
                arm.shootLowSubwoofer();
                wrist.shootLowSubwoofer();
            // if in high mode (low mode false), set the arm and wrist position to shooting high
            } else {
                arm.shootHighSubwoofer();
                wrist.shootHighSubwoofer();
            }
            state = State.ShootingSubwoofer;
        }
    }

    /**
     * Set the arm and wrist to the podium position: low or high
     *
     * @param lowMode true if shooting low and false if shooting high
     */
    public void podiumShoot(boolean lowMode) {
        this.lowMode = lowMode;
        if (safe) return; // do not run if in safe mode
        /* if the state is not already in shooting podium, shoot in low or high mode (specified in parameter lowMode)
        and set the state to ShootingPodium */
        if (state != State.ShootingPodium) {
            // if in low mode, set the arm and wrist position to shooting low
            if (lowMode) {
                arm.shootLowPodium();
                wrist.shootLowPodium();
            // if in high mode (low mode false), set the arm and wrist position to shooting high
            } else {
                arm.shootHighPodium();
                wrist.shootHighPodium();
            }
            state = State.ShootingPodium;
        }
    }

    /**
     * Set the arm and wrist to the amp position
     */
    public void amp() {
        if (safe) return; // do not run if in safe mode
        // if not already in amping position, set the arm and wrist position to amp and set the state to Amping
        if (state != State.Amping) {
            arm.amp();
            wrist.amp();
            state = State.Amping;
        }
    }

    /**
     * Set safe mode to true and set the arm and wrist to intaking
     */
    public void enableSafeMode() {
        intake();
        safe = true;
    }

    /**
     * Set safe mode to false
     */
    public void disableSafeMode() {
        safe = false;
    }

    @Override
    public void periodic() {
        if (state == null) return;
        SmartDashboard.putString("[Arm Wrist] state", state.toString());
        switch (state) {
            case Stowed -> stow();
            case Amping -> amp();
            case Intaking -> intake();
            case ShootingSubwoofer -> subwooferShoot(lowMode);
            case ShootingPodium -> podiumShoot(lowMode);
        }
    }
}
