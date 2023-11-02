package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    //Make an array because the servo

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    DcMotor liftMotor;
    Telemetry telemetry;


    public Lift(HardwareMap hw, Telemetry telemetry){ this(hw, telemetry, "lift_Motor" );}
    public Lift(HardwareMap hw, Telemetry telemetry, String liftName) {

        liftMotor = hw.get(DcMotor.class, liftName);
        this.telemetry = telemetry;
    }


    public void setPower(double power) { liftMotor.setPower(power);}

     // we need to split up the rotation up and the rotation down
    /*We want to have the rotation up of the depsoit to be automatic once the lift reaches half way up
      To bring the depsoit back to its orginal state, we will use a teleOP

     */

     public double getLiftPosition(){
        return liftMotor.getCurrentPosition();
    }

}
