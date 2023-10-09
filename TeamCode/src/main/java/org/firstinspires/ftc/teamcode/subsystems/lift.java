package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.robotcore.hardware.DcMotor;

public class lift {
    DcMotor liftMotor;
    public lift(){
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
    }

    public void armUp() {
        liftMotor.setPower(0.5);
    }
    public void armDown(){
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
    }
}
