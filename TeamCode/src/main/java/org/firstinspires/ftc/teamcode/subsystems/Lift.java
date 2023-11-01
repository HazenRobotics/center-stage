package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
enum Enum
{
     half;
     //enums are interesting
    public double halfwayUpTheLift()
    {
        return 0.5;
    }

}
public class Lift {
    //Make an array because the servo

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    DcMotor liftMotor;
    Deposit deposit = new Deposit(HardwareMap);
    Telemetry telemetry;
    public Lift(HardwareMap hw, Telemetry telemetry){ this(hw, telemetry, "lift_Motor" );}
    public Lift(HardwareMap hw, Telemetry telemetry, String liftName) {

        liftMotor = hw.get(DcMotor.class, liftName);
        this.telemetry = telemetry;
    }


    public void setPower(double power) { liftMotor.setPower(power);}

     public void rotateDeposit(double position){
        if()

    }

}
