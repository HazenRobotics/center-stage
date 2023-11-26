package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private boolean isSettling;
    public static final double SETTLING_POWER=0.5;

    Telemetry telemetry;
    DcMotor intakeMotor;
    CRServo wheelServo;
    Servo deploymentServo;

    //    boolean[] sensorDetectArray = {false, false};
    IntakeCapacity intakeCapacity = IntakeCapacity.EMPTY; //default state, nothing inside
    private double adjustIncrement;
    private double servoPos;
    private double motorPower;
    private boolean reverse = true;

    DeploymentState deploymentState = DeploymentState.FOLDED;

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
        this(hw, t, "intake", "deployIntake");
    }

    public Intake(HardwareMap hw, Telemetry t, String motorName, String deploymentServoName) {
        intakeMotor = hw.get(DcMotor.class, motorName);
        intakeMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        wheelServo = hw.get( CRServo.class, "wheelServo ");
        wheelServo.setDirection( DcMotorSimple.Direction.REVERSE );
        deploymentServo = hw.get(Servo.class, deploymentServoName);
        telemetry = t;
        adjustIncrement = 0.02;
    }

    public IntakeCapacity getIntakeState() {
        return intakeCapacity;
    }

    public DeploymentState getDeploymentState() {
        return deploymentState;
    }

    public void setAdjustIncrement(double increment) {
        adjustIncrement = increment;
    }
    public void setDeployPos( double pos ) {
        servoPos = Range.clip( pos, DeploymentState.FULLY_DEPLOYED.getPosition(), DeploymentState.FOLDED.getPosition());
        deploymentServo.setPosition( servoPos );
    }
    public void foldIntake( ) {
        deploymentState = DeploymentState.FOLDED;
        setDeployPos( DeploymentState.FOLDED.getPosition() );
        reverse = true;
        setIntakeMotorPower( 0 );
        wheelServo.setPower( 0 );
    }
    public void deployIntake( double powerMultiplier ) {
        deploymentState = DeploymentState.FULLY_DEPLOYED;
        setDeployPos( DeploymentState.FULLY_DEPLOYED.getPosition() );
        reverse = !reverse;
        setIntakeMotorPower( (reverse ? -0.8 : 0.8) * powerMultiplier );
        wheelServo.setPower( (reverse ? -1 : 1) );
    }

    public void adjustUp() {
        servoPos += adjustIncrement;
        setDeployPos( servoPos );
    }

    public void adjustDown() {
        servoPos -= adjustIncrement;
        setDeployPos( servoPos );
    }
    public void setIsSettling(boolean settling) {
        isSettling=settling;
    }
    public boolean getIsSettling() {
        return isSettling;
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
