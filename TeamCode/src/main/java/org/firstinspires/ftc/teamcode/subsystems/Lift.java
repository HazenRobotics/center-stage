package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    DcMotor liftMotor;
    Telemetry telemetry;
    public Lift(HardwareMap hwMap, Telemetry t){
        telemetry = t;
        liftMotor = hwMap.get(DcMotor.class, 'lift');
    }

    public void armUp() {
        liftMotor.setPower(0.5);

    }
    public void armDown(){
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public double ticksToIN(double ticks) {
        return (3.5*Math.PI)*(ticks*28);
    }
}
