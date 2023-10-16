package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeDeploymentWheel {
    Servo deployment;
    Telemetry telemetry;

    IntakeWheelState state = IntakeWheelState.DOWN; //default state

    public enum IntakeWheelState {
        DOWN(0),
        UP(0.5);
        private final double position;
        IntakeWheelState(double pos) {
            position = pos;
        }

        public double getPosition() {
            return position;
        }
    }

    public IntakeDeploymentWheel(HardwareMap hardwareMap, Telemetry t, String servoName) {
        deployment = hardwareMap.get(Servo.class, servoName);
        telemetry = t;
    }

    public IntakeWheelState getState() {
        return state;
    }

    public void setPos() {
        deployment.setPosition(state.getPosition());
    }

    public void addTelemetry() {
        telemetry.addData("Intake Wheel State: ", getState());
    }


}
