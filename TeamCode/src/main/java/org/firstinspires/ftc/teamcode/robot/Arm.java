package org.firstinspires.ftc.teamcode.robot;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotor liftMotor1;
    private DcMotor liftMotor2;

    private CRServo gripServo;

    private final int maxArmHeight = -3070;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        liftMotor1 = hardwareMap.get(DcMotor.class,"LiftMot1");
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor2 = hardwareMap.get(DcMotor.class,"LiftMot2");
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        gripServo = hardwareMap.get(CRServo.class, "GripServ");
    }

    public void driveArm(double power) {
        liftMotor1.setPower(power);
        liftMotor2.setPower(-power);

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

    public void gripTest() {
        telemetry.addLine("Testing Gripper");
        telemetry.update();

        gripServo.setDirection(CRServo.Direction.FORWARD);
        gripServo.setPower(0.5);


//        sleep(10000);
    }

    public void gripper(boolean intake, boolean output) {
        gripServo.setDirection(CRServo.Direction.FORWARD);
        if(intake)
            gripServo.setPower(1.0);
        else if (output)
            gripServo.setPower(-1.0);
        else
            gripServo.setPower(0);
    }


    }
