package org.firstinspires.ftc.teamcode.robot;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotor liftMotor;
    private CRServo gripServo;

    private final int maxArmHeight = -3070;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        liftMotor = hardwareMap.get(DcMotor.class,"LiftMot");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripServo = hardwareMap.get(CRServo.class, "GripServ");
    }

    public void driveArm(double power) {
        liftMotor.setPower(power);
    }

    public void calibrateArm() {
        //-3072 is max arm height
        telemetry.addLine("Testing Arm");
        telemetry.update();

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(3000);

        while(true) {
            telemetry.addData("Arm Pos:", liftMotor.getCurrentPosition());
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

        gripServo.setDirection(DcMotorSimple.Direction.FORWARD);
        gripServo.setPower(0.5);


        sleep(10000);
       }
    }
