package org.firstinspires.ftc.teamcode.tests;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@Config
@TeleOp
public class SwervePodCalibration extends LinearOpMode {

    AxonSwervePod[] pods;

    String[] motorNames;
    boolean[] motorReversed;
    String[] servoNames;
    boolean[] servoReversed;
    String[] encoderNames;
    double[] encoderOffsets;

    int selection;

    GamepadEvents controller1;

    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);

        motorNames = new String[] {"FLM", "BLM", "FRM", "BRM"};
        motorReversed = new boolean[] {false, false, false, false };
        servoNames = new String[] {"FLS", "BLS", "FRS", "BRS"};
        servoReversed = new boolean[] {true, true, true, true };
        encoderNames = new String[] {"FLE", "BLE", "FRE", "BRE"};
        encoderOffsets = new double[] {5.15, 4.265, 5.31, 6.18};
        pods = new AxonSwervePod[4];

        for (int i = 0; i < 4; i++) {
            pods[i] = new AxonSwervePod(hardwareMap, motorNames[i], servoNames[i], encoderNames[i]);
            pods[i].setOffset( encoderOffsets[i] );
            if ( motorReversed[i] )
                pods[i].reverseMotor();
            if ( servoReversed[i] )
                pods[i].reverseServo();
        }

        telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

        waitForStart();

        while (opModeIsActive()) {

            if (controller1.dpad_left.onPress())
                selection--;
            else if (controller1.dpad_right.onPress())
                selection++;

            selection = clamp(selection, 0, 3);

            pods[selection].setDrivePower( -gamepad1.left_stick_y );
            pods[selection].setRotatePower( gamepad1.right_stick_x );

            updateTelemetry();
            controller1.update();
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Currently Selected", motorNames[selection]);

        for (int i = 0; i < 4; i++) {
            telemetry.addData("pod " + motorNames[i], pods[i].getDriveVelo());
            telemetry.addData(motorNames[i] + " motor speed", pods[i].getDriveVelo());
            telemetry.addData(motorNames[i] +" angle ", pods[i].getAngle());
            telemetry.addLine();
        }

        telemetry.update();
    }
}
