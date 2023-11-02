package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
 enum Enum
{

   HALF;
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
    //Change halfEnum to a different name
    Enum halfEnum= Enum.HALF;
    public Lift(HardwareMap hw, Telemetry telemetry){ this(hw, telemetry, "lift_Motor" );}
    public Lift(HardwareMap hw, Telemetry telemetry, String liftName) {

        liftMotor = hw.get(DcMotor.class, liftName);
        this.telemetry = telemetry;
    }


    public void setPower(double power) { liftMotor.setPower(power);}

     // we need to split up the rotation up and the rotation down
     public void rotateDeposit(){

         int rotatePress = 0;
        if(liftMotor.getCurrentPosition() >= halfEnum.halfwayUpTheLift())
        {
            if(rotatePress == 0){
                deposit.rotateDepositUp();
                rotatePress++;
                //rotate down
            }else if(rotatePress == 1){
                deposit.rotateDepositDown();
                rotatePress = 0;
            }
        }

    }

}
