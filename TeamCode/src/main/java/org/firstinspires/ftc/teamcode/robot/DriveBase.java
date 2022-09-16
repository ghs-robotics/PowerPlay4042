package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveBase {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private DcMotor LFDrive;
    private DcMotor LRDrive;
    private DcMotor RFDrive;
    private DcMotor RRDrive;

    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        LFDrive = hardwareMap.get(DcMotor .class,"LFDrive");
        LRDrive = hardwareMap.get(DcMotor .class,"LRDrive");
        RFDrive = hardwareMap.get(DcMotor .class,"RFDrive");
        RRDrive = hardwareMap.get(DcMotor .class,"RRDrive");

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
        LFDrive.setPower(lf);
        LRDrive.setPower(lr);
        RFDrive.setPower(rf);
        RRDrive.setPower(rr);
    }
}
