package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    Servo angler, release;

    public enum ReleaseStates {
        RETRACTED(0.582), DROP_ONE(0.636), EXTENDED(0.816);
        private final double position;
        private ReleaseStates (double pos) {
            position = pos;
        }
        double getPosition() {
            return position;
        }
    }

    public Outtake(HardwareMap hw, String releaseName, String anglerName) {
        release = hw.get(Servo.class, releaseName);
        angler = hw.get(Servo.class, anglerName);
    }

    public void setReleasePosition (ReleaseStates state) {
        release.setPosition(state.getPosition());
    }

    public double getReleasePosition() {
        return release.getPosition();
    }
}
