package org.firstinspires.ftc.teamcode.robot;

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

    //private CRServo gripServo;
    public Servo gripServo;

    private final int maxArmHeight = 1100;

    //TODO these numbers need to be added
    private final int highPole = maxArmHeight - 100;
    private final int middlePole = 2 * highPole / 3;
    private final int lowPole = highPole / 3;


    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "LiftMot1");
        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);

        liftMotor2 = hardwareMap.get(DcMotorEx.class, "LiftMot2");
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);

        gripServo = hardwareMap.get(Servo.class, "GripServ");
        gripServo.setDirection(Servo.Direction.FORWARD);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void resetLiftPos(boolean reset) {
        if (reset) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void driveArm(double power) {
        double current = liftMotor1.getCurrentPosition();

        if (current >= maxArmHeight - 100)
            power *= (maxArmHeight - current) / 100;
        else if ((current >= maxArmHeight && power > 0) || (current <= 0 && power < 0))
            power = 0.1;


        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
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
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor1.setTargetPosition(targetPos);
            liftMotor2.setTargetPosition(targetPos);
            driveArm(1);
        } else {
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void runGripper(boolean inMax, boolean outMax, boolean inLimited, boolean outLimited) {
        double current = gripServo.getPosition();
        double twentyDeg = 20 / 300;

        if (inLimited)
            gripServo.setPosition(current -twentyDeg);
        else if (outLimited)
            gripServo.setPosition(current +twentyDeg );
        else if (inMax)
            gripServo.setPosition(Servo.MIN_POSITION);
        else if (outMax)
            gripServo.setPosition(Servo.MAX_POSITION);
    }

    public String gripperStatus(){
        if (gripServo.getPosition() < 0.5)
            return "cannot intake";
        else
            return "cannot drop cone";
    }
}
    //CRServo code
//    public void gripTest() {
//        telemetry.addLine("Testing Gripper");
//        telemetry.update();
//
//        gripServo.setDirection(CRServo.Direction.FORWARD);
//        gripServo.setPower(0.5);
//
//
////        sleep(10000);
//    }
//
//    public void gripper( boolean posBtn, boolean negBtn ) {
//        gripServo.setDirection(CRServo.Direction.FORWARD);
//
//        int pos = posBtn ? 1 : 0;
//        int neg = negBtn ? 1 : 0;
//
//        gripServo.setPower( pos - neg );
//    }
