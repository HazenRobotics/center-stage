package org.firstinspires.ftc.teamcode.tests;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.AxonSwervePod;
import org.firstinspires.ftc.teamcode.utils.Field;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

import java.util.List;

@Config
@TeleOp
@Disabled
public class SwervePodCalibration extends LinearOpMode {

    AxonSwervePod[] pods;

    String[] motorNames;
    boolean[] motorReversed;
    String[] servoNames;
    boolean[] servoReversed;
    String[] encoderNames;
    double[] encoderOffsets;

    int selection;

    public List<LynxModule> hubs;

    GamepadEvents controller1;

    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);

        motorNames = new String[] {"FLM/paraLEnc", "BLM/perpEnc", "FRM/liftEnc", "BRM/paraREnc"};
        motorReversed = new boolean[] {false, false, false, false };
        servoNames = new String[] {"FLS", "BLS", "FRS", "BRS"};
        servoReversed = new boolean[] {false, false, false, false };
        encoderNames = new String[] {"FLE", "BLE", "FRE", "BRE"};
        encoderOffsets = new double[] {0, 0, 0, 0};
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

        hubs = hardwareMap.getAll( LynxModule.class );
        hubs.get( 0 ).setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );
        hubs.get( 1 ).setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );

        waitForStart();

        while (opModeIsActive()) {

            if (controller1.dpad_left.onPress())
                selection--;
            else if (controller1.dpad_right.onPress())
                selection++;
            else if( controller1.b.onPress() )
                pods[selection].setOffset(pods[selection].getAngle());
            else if( controller1.y.onPress() )
                pods[selection].setOffset(0);


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
            telemetry.addData( motorNames[i] + " offset", pods[i].getOffset() );
            telemetry.addData( motorNames[i] + " amps", pods[i].getDriveAmp() );
            telemetry.addLine();
        }

        telemetry.addData( "CH amps", hubs.get( 0 ).getCurrent( CurrentUnit.AMPS ) );
        telemetry.addData( "EH amps", hubs.get( 1 ).getCurrent( CurrentUnit.AMPS ) );

        telemetry.update();
    }
}
