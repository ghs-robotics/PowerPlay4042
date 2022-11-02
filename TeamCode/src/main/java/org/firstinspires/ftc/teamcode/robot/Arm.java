package org.firstinspires.ftc.teamcode.robot;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;

    //private CRServo gripServo;
    private Servo gripServo;

    private final int maxArmHeight = 3070;

    //TODO these numbers need to be added
    private final int highPole = maxArmHeight;
    private final int middlePole = 2000;
    private final int lowPole = 1000;


    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        liftMotor1 = hardwareMap.get(DcMotorEx.class,"LiftMot1");
        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        liftMotor2 = hardwareMap.get(DcMotorEx.class,"LiftMot2");
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        gripServo = hardwareMap.get(Servo.class, "GripServ");
        gripServo.setDirection(Servo.Direction.FORWARD);
    }

    public void resetLiftPos(boolean reset){
        if (reset) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void driveArm(double power) {
        int current = liftMotor1.getCurrentPosition();

        if ((current >= maxArmHeight && power > 0) || (current <= 0 && power < 0))
            power = 0;

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }

    public void runLiftToPos(boolean reset, boolean low, boolean mid, boolean high){
        int targetPos = liftMotor1.getCurrentPosition();

        if (low)
            targetPos = lowPole;
        else if (mid)
            targetPos = middlePole;
        else if (high)
            targetPos = highPole;
        else if (reset)
            targetPos = 0;

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setTargetPosition(targetPos);
        liftMotor2.setTargetPosition(targetPos);
    }

    public void runGripper(boolean in, boolean out){
        if (in)
            gripServo.setPosition(Servo.MIN_POSITION);
        else if (out)
            gripServo.setPosition(Servo.MAX_POSITION);
    }

    public void calibrateArm() {
        //-3072 is max arm height
        telemetry.addLine("Testing Arm");
        telemetry.update();

        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(3000);

        while(true) {
            telemetry.addData("Arm Pos Read 1:", liftMotor1.getCurrentPosition());
            telemetry.addData("Arm Pos Read 2:", liftMotor2.getCurrentPosition());
            telemetry.update();
            sleep(200);
        }

//        liftMotor.setTargetPosition(-3000);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        liftMotor.setPower(1.0);
//
//        while(liftMotor.getCurrentPosition() != -3000) {
//            telemetry.addLine("Moving to Pos");
//            telemetry.update();
//        }
//
//        sleep(1000);
//
//        liftMotor.setTargetPosition(0);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        liftMotor.setPower(0.5);


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
}
