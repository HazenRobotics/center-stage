package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    Telemetry telemetry;
    DcMotor intakeMotor;
    Servo deploymentServo;
    //    boolean[] sensorDetectArray = {false, false};
    IntakeCapacity intakeCapacity = IntakeCapacity.EMPTY; //default state, nothing inside
    IntakeBreakBeamSensor breakBeamTop, breakBeamBottom, breakBeamIntake;
    private double adjustIncrement;
    private double servoPos;
    private double motorPower;

    public enum IntakeCapacity {
        EMPTY, ONE_PIXEL, FULL, OVERFLOW
    }
    public enum DeploymentState {
        FOLDED(0.655), TOP_PIXEL(0.235), SECOND_PIXEL(0.185), FULLY_DEPLOYED(0.215);
        public final double position;
        DeploymentState(double pos) {
            position = pos;
        }
        public double getPosition() {
            return position;
        }
    }

    public Intake(HardwareMap hw, Telemetry t) {
        this(hw, t, "intake", "deployIntake",
                "BB-Top", "BB-Bot", "BB-In");
    }

    public Intake(HardwareMap hw, Telemetry t, String motorName, String deploymentServoName, String breakBeamTopName, String breakBeamBottomName, String breakBeamIntakeName) {
        intakeMotor = hw.get(DcMotor.class, motorName);
        intakeMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        deploymentServo = hw.get(Servo.class, deploymentServoName);
        breakBeamTop = new IntakeBreakBeamSensor(hw, t, breakBeamTopName);
        breakBeamBottom = new IntakeBreakBeamSensor(hw, t, breakBeamBottomName);
        breakBeamIntake = new IntakeBreakBeamSensor(hw, t, breakBeamIntakeName);
        telemetry = t;
        adjustIncrement = 0.02;
    }

    public IntakeCapacity getIntakeState() {
        return intakeCapacity;
    }

    public void setAdjustIncrement(double increment) {
        adjustIncrement = increment;
    }
    public void setDeployPos( double pos ) {
        servoPos = Range.clip( pos, DeploymentState.FULLY_DEPLOYED.getPosition(), DeploymentState.FOLDED.getPosition());
        deploymentServo.setPosition( servoPos );
    }
    public void foldIntake( ) {
        setDeployPos( DeploymentState.FOLDED.getPosition() );
        setIntakeMotorPower( 0 );
    }
    public void deployIntake( double powerMultiplier ) {
        setDeployPos( DeploymentState.FULLY_DEPLOYED.getPosition() );
        setIntakeMotorPower( (motorPower < 0 ? 0.8 : -0.8) * powerMultiplier );
    }

    public void adjustUp() {
        servoPos += adjustIncrement;
        setDeployPos( servoPos );
    }

    public void adjustDown() {
        servoPos -= adjustIncrement;
        setDeployPos( servoPos );
    }

    /**
     * updates the status of the intakeCapacity variable depending on how many pixels are detected.
     */
//    public void updateIntakeCapacity() {
//        updatePixelColorArray();
//        breakBeam.updateBeamState();
//
//        boolean pixelInSlotOne = cs1.getPixelColour() != Field.Pixel.NONE;
//        boolean pixelInSlotTwo = cs2.getPixelColour() != Field.Pixel.NONE;
//        boolean pixelInIntake = breakBeam.getBeamState();
//
//        if (pixelInSlotOne && pixelInSlotTwo && pixelInIntake)
//            intakeCapacity = IntakeCapacity.OVERFLOW;
//        else if ((pixelInIntake && pixelInSlotOne) || (pixelInSlotOne && pixelInSlotTwo))
//            intakeCapacity = IntakeCapacity.FULL;
//        else if (pixelInIntake || pixelInSlotOne)
//            intakeCapacity = IntakeCapacity.ONE_PIXEL;
//        else
//            intakeCapacity = IntakeCapacity.EMPTY;
//    }

    public void setIntakeMotorPower(double power) {
        motorPower = power /* * (intakeCapacity == IntakeCapacity.OVERFLOW ? -1 : 1)*/;
        intakeMotor.setPower( motorPower );
    }

    public void toggleServo(boolean isIntakeServoDisabled, double value)
    {
        //CRServos are related to motors, so I am assuming that they need to be toggled
            if(isIntakeServoDisabled)
            {
                setIntakeMotorPower(value);
                isIntakeServoDisabled = false;
            }else {
                setIntakeMotorPower(0);
                isIntakeServoDisabled = true;
            }

    }

    public double getIntakeMotorPower() {
        return motorPower;
    }

    //retrieves telemetry from sensors, and display current pixel inventory
    public void addTelemetry() {
//        breakBeam.addTelemetry();
//        telemetry.addData("Intake: ", getIntakeState());
//        updatePixelColorArray();
//        telemetry.addData("Pixel Slot 1: ", pixelColorArray[0]);
//        telemetry.addData("Pixel Slot 2: ", pixelColorArray[1]);
        telemetry.addData( "intakePower", motorPower );
        telemetry.addData( "servoPos", servoPos );
    }

}
