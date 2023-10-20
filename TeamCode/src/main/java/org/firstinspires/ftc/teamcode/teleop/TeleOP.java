package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.MecDrive;


public class TeleOP extends LinearOpMode {
    MecDrive bot = new MecDrive();
    Claw claw = new Claw(HardwareMap, Telemetry);
    Lift lift = new Lift(HardwareMap, Telemetry);


    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive())
        {

        }
    }
}
