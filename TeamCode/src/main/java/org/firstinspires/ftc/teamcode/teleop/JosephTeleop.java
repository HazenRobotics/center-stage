package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.ElementType;

public class JosephTeleop {
    ElapsedTime runTime = new ElapsedTime();
    public void init(){
    }
    public void loop() {
        telemetry.addData("Time passed: ", runTime.toString());
    }
}
