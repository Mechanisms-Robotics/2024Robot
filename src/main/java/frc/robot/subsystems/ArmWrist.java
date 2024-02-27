package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmWrist extends SubsystemBase {
    private static final Arm arm = new Arm();
    private static final Wrist wrist = new Wrist();
    private boolean safe = false;
    private enum State {
        Stowed,
        Intaking,
        Amping,
        ShootingSubwoofer,
        ShootingPodium
    }
    private State state = State.Stowed;
    public void stow() {
        if (safe) return;
        if (!state.equals(State.Stowed)) {
            arm.stow();
            wrist.stow();
            state = State.Stowed;
        }
    }
    public void intake() {
        if (!state.equals(State.Intaking)) {
            arm.intake();
            wrist.intake();
            state = State.Intaking;
        }
    }
    public void subwooferShoot(boolean lowMode) {
        if (safe) return;
        if (!state.equals(State.ShootingSubwoofer)) {
            if (lowMode) {
                arm.shootLowSubwoofer();
                wrist.shootLowSubwoofer();

            } else {
                arm.shootHighSubwoofer();
                wrist.shootHighSubwoofer();
            }
            state = State.ShootingSubwoofer;
        }
    }
    public void podiumShoot(boolean lowMode) {
        if (safe) return;
        if (!state.equals(State.ShootingPodium)) {
            if (lowMode) {
                arm.shootLowPodium();
                wrist.shootLowPodium();

            } else {
                arm.shootHighPodium();
                wrist.shootHighPodium();
            }
            state = State.ShootingPodium;
        }
    }

    public void amp() {
        if (safe) return;
        if (!state.equals(State.Amping)) {
            arm.amp();
            wrist.amp();
            state = State.Amping;
        }
    }

    public void enableSafeMode() {
        intake();
        safe = true;
    }
    public void disableSafeMode() {
        safe = false;
    }
}
