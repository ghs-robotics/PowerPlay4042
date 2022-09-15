package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveBase {
    public static DcMotor leftFrontDrive;
    public static DcMotor rightFrontDrive;
    public static DcMotor leftRearDrive;
    public static DcMotor rightRearDrive;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public DriveBase (HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "LRDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "RRDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    public static void calculateDrivePower(double x, double y, double r){
        r = -r;
        double lf = r - x + y;
        double lr = r + x + y;
        double rf = r - x - y;
        double rr = r + x - y;

        sendDrivePower(lf, lr, rf, rr);
    }

    public static void sendDrivePower(double lf, double lr, double rf, double rr){
        leftFrontDrive.setPower(lf);
        leftRearDrive.setPower(lr);
        rightFrontDrive.setPower(rf);
        rightRearDrive.setPower(rr);
    }

}
