package org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.scarabtuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.Pose2D;
import org.firstinspires.ftc.teamcode.utils.mercuriallocalizer.geometry.angle.AngleDegrees;

@Autonomous
@Config
@Disabled
public class PositionPIDTuning extends LinearOpMode {
	KhepriBot robot;
	FtcDashboard dashboard;
	public static double Fp, Fi, Fd, Hp, Hi, Hd;
	public static double x, y, heading;

	enum Mode {
		PID_TUNING,
		DRIVER_CONTROLLED
	}

	Mode mode = Mode.PID_TUNING;
	@Override
	public void runOpMode( ) throws InterruptedException {
		robot = new KhepriBot( hardwareMap, telemetry );
		robot.setupAutoTracker( new Pose2D( 0, 0, new AngleDegrees( 0 ).getTheta()) );
//		telemetry.setMsTransmissionInterval( 25 );

		dashboard = FtcDashboard.getInstance();
//		dashboard.setTelemetryTransmissionInterval(25);

		telemetry = new MultipleTelemetry( telemetry, FtcDashboard.getInstance( ).getTelemetry( ) );

		waitForStart();

		while (opModeIsActive()) {
			robot.update();

			if( gamepad1.a ) {
				mode = Mode.PID_TUNING;
			} else if( gamepad1.x ) {
				mode = Mode.DRIVER_CONTROLLED;
			} else if( gamepad1.y ) {
				heading = 180;
			} else if ( gamepad1.b ) {
				heading = 0;
			}

			switch(mode) {
				case PID_TUNING:
//					robot.XController.setPID( Fp, Fi, Fd );
//					robot.YController.setPID( Fp, Fi, Fd );
//					robot.headingController.setPID( Hp, Hi, Hd );
					robot.goToPoint( x, y, Math.toRadians( heading ) );
					break;
				case DRIVER_CONTROLLED:
					double drive = -gamepad1.left_stick_y;
					double strafe = gamepad1.left_stick_x;
					double rotate = gamepad1.right_stick_x;
					robot.drive.drive(drive, strafe, rotate);
					break;
			}


			telemetry.addData( "x error", robot.XController.getPositionError() );
			telemetry.addData( "y error", robot.YController.getPositionError() );
			telemetry.addData( "heading error", Math.toDegrees( robot.autoHeadingController.getPositionError() ) );
			telemetry.update();
		}
	}

}
