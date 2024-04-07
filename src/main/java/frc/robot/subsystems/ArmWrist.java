package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the arm and the wrist as one subsystem because the states are the same.
 */
public class ArmWrist extends SubsystemBase {
    private final Arm arm;
    private final Wrist wrist;
    private boolean safe = false;

    public enum State {
        /** Holds the arm up as a safe default position */
        Stowed,
        /** Lowers the arm and the wrist to the ground to pick up a note */
        Intaking,
        /** Raises the arm and points the shooter down to roll the note into the amp */
        Amping,
        /** Raises or lowers the arm to the subwoofer position for shooting right at the subwoofer */
        ShootingSubwoofer,
        /** Raises the arm for the shooting podium position */
        ShootingPodium,
        /** Aiming, neither shooting and the podium nor the subwoofer, using vision to tilt the wrist for aiming */
        Aiming,
        /**
         * Raises the arm and points the wrist at the ground at a slight angle for shuttleing the note to the
         * other side
         */
        Shuttleing,
        /** Raises the arm and wrist just high enough for the hooks to go over the chain */
        PrepClimb,
        /** Brings the arm and the wrist down to pull on the chain and bring the robot up */
        Climb
    }
    private State state;
    public State getState() {
        return state;
    }
    private boolean lowMode = false;

    public ArmWrist(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;
    }

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
        if (safe) return; // do not run if in safe mode
        /* if the state is not already in shooting subwoofer, shoot in low or high mode (specified in parameter lowMode)
        and set the state to ShootingSubwoofer */
        if (state != State.ShootingSubwoofer || lowMode != this.lowMode) {
            // if in low mode, set the arm and wrist position to shooting low
            if (lowMode) {
                SmartDashboard.putString("[Arm Wrist] shooting",  "subwoofer low");
                arm.shootLowSubwoofer();
                wrist.shootLowSubwoofer();
            // if in high mode (low mode false), set the arm and wrist position to shooting high
            } else {
                SmartDashboard.putString("[Arm Wrist] shooting",  "subwoofer high");
                arm.shootHighSubwoofer();
                wrist.shootHighSubwoofer();
            }
            state = State.ShootingSubwoofer;
        }
        this.lowMode = lowMode;
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
                SmartDashboard.putString("[Arm Wrist] shooting",  "podium low");
                arm.shootLowPodium();
                wrist.shootLowPodium();
            // if in high mode (low mode false), set the arm and wrist position to shooting high
            } else {
                SmartDashboard.putString("[Arm Wrist] shooting",  "podium high");
                arm.shootHighPodium();
                wrist.shootHighPodium();
            }
            state = State.ShootingPodium;
        }
    }

    /** Set the arm and wrist to the amp position */
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
     * Sets the arm and wrist to their positions for shuttling the note across the floor.
     * Useful for transporting notes to your side of the field.
     */
    public void shuttle() {
        if (safe) return; // do not run if in safe mode
        if (state != State.Shuttleing) {
            arm.shuttle();
            wrist.shuttle();
            state = State.Shuttleing;
        }
    }

    /** Set safe mode to true and set the arm and wrist to intaking cuh */
    public void enableSafeMode() {
        intake();
        safe = true;
    }

    /** Set safe mode to false */
    public void disableSafeMode() {
        safe = false;
    }

    /**
     * Sets the arm and the wrist positions to the position for preparing to climb.
     * This position moves the hooks on gerald above the chain, ready to pull on it.
     */
    public void prepClimb() {
        if (safe) return;
        if (state != State.PrepClimb) {
            arm.prepClimb();
            wrist.prepClimb();
            state = State.PrepClimb;
        }
    }

    /**
     * Set the arm and wrist to the position for climbing.
     * This position brings the arm and wrist down, pulling on the chain and bringing the robot up.
     */
    public void climb() {
        if (safe) return;
        if (state != State.Climb) {
            arm.climb();
            wrist.climb();
            state = State.Climb;
        }
    }

    /**
     * Aims the arm and wrist at the given angles
     *
     * @param desiredArmRotation angle of the arm
     * @param desiredWristRotation angle of the wrist
     */
    public void aim(Rotation2d desiredArmRotation, Rotation2d desiredWristRotation) {
        if (safe) return;
        if (state != State.Aiming) state = State.Aiming;
        arm.aim(desiredArmRotation);
        wrist.aim(desiredWristRotation);
    }

    /** @return true if both the arm and the wrist are at the desired angle, otherwise false */
    public boolean aimed() {
        return arm.aimed() && wrist.aimed();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[Arm Wrist] aimed", aimed());
        if (state == null) return;
        SmartDashboard.putString("[Arm Wrist] state", state.toString());
        switch (state) {
            case Stowed -> stow();
            case Amping -> amp();
            case Intaking -> intake();
            case ShootingSubwoofer -> subwooferShoot(lowMode);
            case ShootingPodium -> podiumShoot(lowMode);
            case Shuttleing -> shuttle();
            case PrepClimb -> prepClimb();
            case Climb -> climb();
        }
    }
}
