package org.firstinspires.ftc.teamcode.robot;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx liftMotor1;
    public DcMotorEx liftMotor2;

    public CRServo gripServo;

    public Servo brakeServo;

    private final int maxArmHeight = 1100;

    private boolean lowerLimitBtnPressedLastFrame = false;
    private boolean lowerLimitActive = true;
    private double armPosError = 0;

    private final double AutoArmStopDist = 5;

    private double brakePos = 0.59;
    private double letRunPos = 0.565;

    //TODO these numbers need to be added
    private final int highPole = maxArmHeight - 100;
    private final int middlePole = 2 * highPole / 3;
    private final int lowPole = highPole / 3;


    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        //liftMotor1 = hardwareMap.get(DcMotor.class, "LiftMot1");
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "LiftMot1");
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);

        //liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMot2");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "LiftMot2");
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        gripServo = hardwareMap.get(CRServo.class, "GripServ");
        gripServo.setDirection(CRServo.Direction.FORWARD);

        brakeServo = hardwareMap.get(Servo.class, "brake");

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getPoleHeight(int pole) {
        // 0 = ground | 1 = low | 2 = middle | 3 = high

        switch (pole) {
            case 1: return lowPole;
            case 2: return middlePole;
            case 3: return highPole;
        }
        return 0;
    }

    public void driveArm(double input) {
        double power = input;

        double crntPos = liftMotor1.getCurrentPosition();

        if (brakeServo.getPosition() < brakePos - 0.02){
            double maxArmHeightPlusError = maxArmHeight + armPosError;
            if (crntPos >= maxArmHeightPlusError - 100 && power > 0) {
                power *= (maxArmHeightPlusError - crntPos) / 100;
                if (power <= 0.05) power = 0.05;
            }
            else if (lowerLimitActive && crntPos <= armPosError && power < 0)
                power = 0;
            else if (crntPos >= maxArmHeightPlusError && power > 0)
                power = 0.1;

            liftMotor1.setPower(power);
            liftMotor2.setPower(power);
        }

        if (input == 0) brakeArmManual(true);
        else brakeArmManual(false);
    }

    public void brakeArmAuto(){
        if (liftMotor1.getPower() == 0 || liftMotor2.getPower() == 0) brakeServo.setPosition(brakePos);
        else brakeServo.setPosition(letRunPos);
    }
    private void brakeArmManual(boolean brake) {
        if (brake) brakeServo.setPosition(brakePos);
        else brakeServo.setPosition(letRunPos);
    }
    public void RemoveAndSetLowerLimit(boolean btnPressed) {
        if (btnPressed) lowerLimitActive = false;
        else {
            lowerLimitActive = true;
            if (lowerLimitBtnPressedLastFrame) armPosError = liftMotor1.getCurrentPosition();
        }
        lowerLimitBtnPressedLastFrame = btnPressed;
    }

    public void runLiftToPos(boolean reset, boolean low, boolean mid, boolean high) {
        int targetPos = liftMotor1.getCurrentPosition();
        int error = liftMotor1.getCurrentPosition() - liftMotor2.getCurrentPosition();

        if (low)
            targetPos = lowPole;
        else if (mid)
            targetPos = middlePole;
        else if (high)
            targetPos = highPole;
        else if (reset)
            targetPos = 0;

        if (reset || low || mid || high) {
            liftMotor1.setTargetPosition(targetPos);
            liftMotor2.setTargetPosition(targetPos);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveArm(1);
        } else {
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        brakeArmAuto();
    }
    public void calibrateLift() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("moton 1 pos: ", liftMotor1.getCurrentPosition());
        telemetry.addData("moton 2 pos: ", liftMotor2.getCurrentPosition());

        telemetry.update();
    }
//
//    public void runGripperRestricted(boolean inMax, boolean outMax, boolean inLimited, boolean outLimited) {
//        double current = gripServo.getPosition();
//        double twentyDeg = 20 / 300;
//
//        if (inLimited)
//            gripServo.setPosition(current -twentyDeg);
//        else if (outLimited)
//            gripServo.setPosition(current +twentyDeg );
//        else if (inMax)
//            gripServo.setPosition(Servo.MIN_POSITION);
//        else if (outMax)
//            gripServo.setPosition(Servo.MAX_POSITION);
//    }
//
//    public String gripperStatus(){
//        if (gripServo.getPosition() < 0.5)
//            return "cannot intake";
//        else
//            return "cannot drop cone";
//    }

    //CRServo code - previously posbtn and negbtn
    public void runGripperContinuous( boolean intakeBtn, boolean releaseBtn ) {
        gripServo.setDirection(CRServo.Direction.FORWARD);

        int intake = intakeBtn ? 1 : 0;
        int release = releaseBtn ? 1 : 0;

        gripServo.setPower( release - intake );
    }

    public void RotateGripperForDuration(boolean intake, long milliseconds) {
        gripServo.setDirection(CRServo.Direction.FORWARD);

        int dir = intake ? -1 : 1;

        gripServo.setPower( dir );

        sleep(milliseconds);

        gripServo.setPower( 0 );
    }
    public void AutoLiftToPos(int targetPole) {
        int targetHeight = getPoleHeight(targetPole);
        if (targetPole != 0) targetHeight += + 100;

        int crntPos = liftMotor1.getCurrentPosition();
        int error = liftMotor2.getCurrentPosition() - liftMotor2.getCurrentPosition();//add to telemetry?

        double dif = targetHeight - crntPos;
        double absDif = Math.abs( dif );

        liftMotor1.setTargetPosition(targetHeight);
        liftMotor2.setTargetPosition(targetHeight); //account for error?

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (brakeServo.getPosition() > brakePos - 0.02 ) brakeArmManual(false);

        driveArm(0.8);

        while (absDif > AutoArmStopDist) {
            crntPos = liftMotor1.getCurrentPosition();

            dif = targetHeight - crntPos;
            absDif = Math.abs( dif );
        }

        driveArm(0);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (brakeServo.getPosition() < brakePos - 0.005 ) brakeArmManual(true);
    }
}

