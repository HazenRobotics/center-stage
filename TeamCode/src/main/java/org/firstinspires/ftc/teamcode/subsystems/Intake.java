package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Field;

public class Intake {

	Telemetry telemetry;
	DcMotor intakeMotor;

	//    boolean[] sensorDetectArray = {false, false};
	IntakeCapacity intakeCapacity = IntakeCapacity.EMPTY; //default state, nothing inside
	IntakeColourSensor cs1;
	IntakeColourSensor cs2;
	IntakeBreakBeamSensor breakBeam;
	Field.Pixel[] pixelColourArray = new Field.Pixel[2];

	public enum IntakeCapacity {
		EMPTY,
		ONE_PIXEL,
		FULL,
		OVERFLOW
	}

	public Intake( HardwareMap hw, Telemetry t, String motorName, String firstColourSensorName, String secondColourSensorName, String breakBeamSensorName ) {
		intakeMotor = hw.get( DcMotor.class, motorName );
		cs1 = new IntakeColourSensor( hw, t, firstColourSensorName );
		cs2 = new IntakeColourSensor( hw, t, secondColourSensorName );
		breakBeam = new IntakeBreakBeamSensor( hw, t, breakBeamSensorName );
		telemetry = t;
	}

	public Field.Pixel[] getPixelColourArray( ) {
		return pixelColourArray;
	}

	public void updatePixelColourArray( ) {
		cs1.readPixelColour( );
		cs2.readPixelColour( );
		pixelColourArray[0] = cs1.getPixelColour( );
		pixelColourArray[1] = cs2.getPixelColour( );
	}

    public IntakeCapacity getIntakeState( ) {
		return intakeCapacity;
	}

    /**
     * updates the status of the intakeCapacity variable depending on how many pixels are detected.
     */
    public void updateIntakeCapacity( ) {
		updatePixelColourArray( );
		breakBeam.updateBeamState( );

		boolean pixelInSlotOne = cs1.getPixelColour( ) != Field.Pixel.NONE;
		boolean pixelInSlotTwo = cs2.getPixelColour( ) != Field.Pixel.NONE;
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
		updatePixelColourArray();
		telemetry.addData( "Pixel Slot 1: ", pixelColourArray[0] );
		telemetry.addData( "Pixel Slot 2: ", pixelColourArray[1] );
	}

}
