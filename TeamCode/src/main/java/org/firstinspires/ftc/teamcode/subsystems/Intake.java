package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Field;

public class Intake {

	Telemetry telemetry;
	DcMotor intakeMotor; //intake motor

	//    boolean[] sensorDetectArray = {false, false};
	IntakeCapacity intakeCapacity = IntakeCapacity.EMPTY; //default state, nothing inside
	IntakeColorSensor cs1; //first color sensor
	IntakeColorSensor cs2; //second color sensor
	IntakeBreakBeamSensor breakBeam; //break beam sensor
	Field.Pixel[] pixelColorArray = new Field.Pixel[2]; //array for pixel colors

	public enum IntakeCapacity {
		EMPTY,
		ONE_PIXEL,
		FULL,
		OVERFLOW
	}

	public Intake( HardwareMap hw, Telemetry t, String motorName, String firstColorSensorName, String secondColorSensorName, String breakBeamSensorName ) {
		intakeMotor = hw.get( DcMotor.class, motorName );
		cs1 = new IntakeColorSensor( hw, t, firstColorSensorName );
		cs2 = new IntakeColorSensor( hw, t, secondColorSensorName );
		breakBeam = new IntakeBreakBeamSensor( hw, t, breakBeamSensorName );
		telemetry = t;
	}

	public Field.Pixel[] getPixelColorArray( ) {
		return pixelColorArray;
	}

	public void updatePixelColorArray( ) {
		cs1.readPixelColor( );
		cs2.readPixelColor( );
		pixelColorArray[0] = cs1.getPixelColor( );
		pixelColorArray[1] = cs2.getPixelColor( );
	}

    public IntakeCapacity getIntakeState( ) {
		return intakeCapacity;
	}

    /**
     * updates the status of the intakeCapacity variable depending on how many pixels are detected.
     */
    public void updateIntakeCapacity( ) {
		updatePixelColorArray( );
		breakBeam.updateBeamState( );

		boolean pixelInSlotOne = cs1.getPixelColor( ) != Field.Pixel.NONE;
		boolean pixelInSlotTwo = cs2.getPixelColor( ) != Field.Pixel.NONE;
		boolean pixelInIntake = breakBeam.getBeamState( );

		if( pixelInSlotOne && pixelInSlotTwo && pixelInIntake )
			intakeCapacity = IntakeCapacity.OVERFLOW;
		else if( (pixelInIntake && pixelInSlotOne) || (pixelInSlotOne && pixelInSlotTwo) )
			intakeCapacity = IntakeCapacity.FULL;
		else if( pixelInIntake || pixelInSlotOne )
			intakeCapacity = IntakeCapacity.ONE_PIXEL;
		else
			intakeCapacity = IntakeCapacity.EMPTY;
	}

	public void setIntakeMotorPower( double power ) {
		intakeMotor.setPower( power * (intakeCapacity == IntakeCapacity.OVERFLOW ? -1 : 1) );
	}

	public double getIntakeMotorPower( ) {
        return intakeMotor.getPower();
    }

	//retrieves telemetry from sensors, and display current pixel inventory
	public void addTelemetry( ) {
		cs1.getTelemetry( );
		cs2.getTelemetry( );
		breakBeam.addTelemetry( );
		telemetry.addData( "Intake: ", getIntakeState( ) );
		telemetry.addData( "Pixel Slot 1: ", pixelColorArray[0] );
		telemetry.addData( "Pixel Slot 2: ", pixelColorArray[1] );
	}

}
