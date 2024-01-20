package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.function.Sin;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.KhepriBot;
import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingCRServo;
import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingDcMotorEX;
import org.firstinspires.ftc.teamcode.utils.cachinghardwaredevice.CachingServo;

@Config
public class Intake{
    Telemetry telemetry;
    DcMotorEx intakeMotor;
    CRServo wheelServo;
    Servo deploymentServo;
    public enum DeploymentState {
        FOLDED(0.655), TOP_TWO(0.334), FULLY_DEPLOYED(0.2);
        public final double position;
        DeploymentState(double pos) {
            position = pos;
        }
        public double getPosition() {
            return position;
        }
    }

    public enum SpinState {
        IN(0.8), OUT(-1), OFF(0);
        public final double power;
        SpinState(double pow) {
            power = pow;
        }
        public double getPower() {
            return power;
        }
    }

    DeploymentState deploymentState = DeploymentState.FOLDED;
    SpinState spinState = SpinState.OFF;

    public Intake(HardwareMap hw, Telemetry t) {
        this(hw, t, "intake", "deployIntake");
    }

    public Intake(HardwareMap hw, Telemetry t, String motorName, String deploymentServoName) {
        intakeMotor = new CachingDcMotorEX( hw.get(DcMotorEx.class, motorName) );
//        intakeMotor.setDirection( DcMotorSimple.Direction.REVERSE );
        deploymentServo = new CachingServo( hw.get(Servo.class, deploymentServoName) );
        wheelServo = new CachingCRServo( hw.get( CRServo.class, "wheelServo") );
//        wheelServo.setDirection( DcMotorSimple.Direction.REVERSE );
        telemetry = t;
    }

    public DeploymentState getDeploymentState() {
        return deploymentState;
    }

    public double getAngle() {
        return deploymentServo.getPosition();
    }

    public void setDeployPos( DeploymentState state ) {
        deploymentServo.setPosition( state.getPosition() );
    }
    public void setDeployPos( double pos ) {
        deploymentServo.setPosition(
                Range.clip( pos, DeploymentState.FULLY_DEPLOYED.getPosition(), DeploymentState.FOLDED.getPosition())
        );
    }
    public void foldIntake( ) {
        deploymentState = DeploymentState.FOLDED;
        spinState = SpinState.OFF;
    }
    public void deployIntake( ) {
        switch( deploymentState ) {
            case FOLDED:
            case TOP_TWO:
                deploymentState = DeploymentState.FULLY_DEPLOYED;
                spinState = SpinState.IN;
                break;
            case FULLY_DEPLOYED:
                deploymentState = DeploymentState.TOP_TWO;
                spinState = SpinState.OUT;
                break;
        }
    }

    public void update() {
        setDeployPos( deploymentState );
        setIntakeMotorPower( spinState.getPower() * KhepriBot.normalizedPowerMultiplier );
        setIntakeServoPower( spinState.getPower() );
    }
    public void setIntakeMotorPower( double power ) {
        intakeMotor.setPower( power );
    }
    public void setIntakeServoPower(double power) {
        wheelServo.setPower( power );
    }
    public double getIntakeMotorPower() {
        return intakeMotor.getPower( );
    }
    public double getIntakeServoPower() {return wheelServo.getPower(); }

    public double getMotorCurrent() {
        return intakeMotor.getCurrent( CurrentUnit.AMPS ) * getIntakeMotorPower();
    }

    public boolean isReversed() {
        return spinState == SpinState.OUT;
    }
}
