package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    DcMotor liftMotor;
    Telemetry telemetry;
    public Lift(HardwareMap hw, Telemetry t){
        telemetry = t;
        liftMotor = hw.get(DcMotor.class, "lift");
    }
//
    public void armUp(double power) {
        liftMotor.setPower(power);
    }


}
