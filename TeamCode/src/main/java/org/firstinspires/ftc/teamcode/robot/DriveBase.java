package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveBase {

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        frontLeftDrive = hardwareMap.get(DcMotor.class,"FLDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class,"BLDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class,"FRDrive");
        backRightDrive = hardwareMap.get(DcMotor.class,"BRDrive");

    }
    public void calculateDrivePower(double x, double y, double rot){
        rot = -rot;
        double frontLeft = rot - x + y;
        double backLeft = rot + x + y;
        double frontRight = rot - x - y;
        double backRight = rot + x - y;

        sendDrivePower(frontLeft, backLeft, frontRight, backRight);
    }

    public void sendDrivePower(double frontLeft, double backLeft, double frontRight, double backRight){
        //reversed to match motor polarity or something
        frontLeftDrive.setPower(frontLeft);
        backLeftDrive.setPower(backLeft);
        frontRightDrive.setPower(frontRight);
        backRightDrive.setPower(backRight);
    }
}
