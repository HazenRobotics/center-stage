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
	Field.Pixel[] pixelColorArray = new Field.Pixel[2];

	public enum IntakeCapacity {
		EMPTY,
		ONE_PIXEL,
		FULL,
		OVERFLOW
	}

	public Intake( HardwareMap hw, Telemetry t, String motorName, String firstColorSensorName, String secondColorSensorName, String breakBeamSensorName ) {
		intakeMotor = hw.get( DcMotor.class, motorName );
		cs1 = new IntakeColourSensor( hw, t, firstColorSensorName );
		cs2 = new IntakeColourSensor( hw, t, secondColorSensorName );
		breakBeam = new IntakeBreakBeamSensor( hw, t, breakBeamSensorName );
		telemetry = t;
	}

	public Field.Pixel[] getPixelColorArray( ) {
		return pixelColorArray;
	}

	public void updatePixelColorArray( ) {
		cs1.readPixelColour( );
		cs2.readPixelColour( );
		pixelColorArray[0] = cs1.getPixelColour( );
		pixelColorArray[1] = cs2.getPixelColour( );
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
		cs1.getTelemetry();
		cs2.getTelemetry();
		breakBeam.addTelemetry( );
		telemetry.addData( "Intake: ", getIntakeState( ) );
		updatePixelColorArray();
		telemetry.addData( "Pixel Slot 1: ", pixelColorArray[0] );
		telemetry.addData( "Pixel Slot 2: ", pixelColorArray[1] );
	}

}
