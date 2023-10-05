package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "PRBotTest",group = "TeleOp")
public class PRBot extends OpMode {
    GamepadEvents controller;
    DcMotor[] motors=new DcMotor[6];
    String[] motorNames = {"fl","bl","fr","br","leftFly","rightFly"};
    Servo trigger;

    @Override
    public void init( ) {
        for(int i=0;i<motors.length;i++){
            motors[i]= hardwareMap.get(DcMotor.class, motorNames[i]);
        }
        trigger = hardwareMap.get(Servo.class,"trigger");
        trigger.setPosition(0.5);
        motors[2].setDirection( DcMotorSimple.Direction.REVERSE );
        //For some reason the bot does not like it when you reverse this motor
        //motors[3].setDirection( DcMotorSimple.Direction.REVERSE );
        motors[5].setDirection( DcMotorSimple.Direction.REVERSE);
        controller = new GamepadEvents( gamepad1 );

    }

    @Override
    public void loop( ) {
        double drive = controller.left_stick_y;
        double strafe = controller.left_stick_x;
        double rotate = -controller.right_stick_x;

        motors[0].setPower(drive + strafe + rotate);
        motors[1].setPower(drive - strafe + rotate);
        motors[2].setPower(drive - strafe - rotate);
        motors[3].setPower(drive + strafe - rotate);

        double flyWheelSpeed = controller.right_trigger.getTriggerValue();
        motors[4].setPower(flyWheelSpeed);
        motors[5].setPower(flyWheelSpeed);

        //At some point this will probably have some automatic system
        //that will return the trigger back to rest so you don't
        //have to manually "reload" each time.
        if (controller.a.onPress() ) {
            //If trigger is NOT extended
            if( trigger.getPosition() == 0.5)
                trigger.setPosition(1);
            //If trigger is extended
            else if ( trigger.getPosition() == 1)
                trigger.setPosition(0.5);
        }

        controller.update();
    }
}
