package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.util.Encoder;

public class DriveBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private DcMotor leftFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;

    private Encoder leftOdo;
    private Encoder rightOdo;
    private Encoder backOdo;

    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        leftFrontDrive = hardwareMap.get(DcMotor .class,"LFDrive");
        leftRearDrive = hardwareMap.get(DcMotor .class,"LRDrive");
        rightFrontDrive = hardwareMap.get(DcMotor .class,"RFDrive");
        rightRearDrive = hardwareMap.get(DcMotor .class,"RRDrive");

    }
    public void calculateDrivePower(double x, double y, double r){
        r = -r;
        double lf = r - x + y;
        double lr = r + x + y;
        double rf = r - x - y;
        double rr = r + x - y;

        sendDrivePower(lf, lr, rf, rr);
    }

    public void sendDrivePower(double lf, double lr, double rf, double rr){ //reversed to match motor polarity or something
        leftFrontDrive.setPower(lf);
        leftRearDrive.setPower(lr);
        rightFrontDrive.setPower(rf);
        rightRearDrive.setPower(rr);
    }
}
