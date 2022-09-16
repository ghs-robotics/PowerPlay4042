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
    public void setDrivePower(){
        LFDrive.setPower(1);
        LRDrive.setPower(1);
        RFDrive.setPower(-1);
        RRDrive.setPower(-1);
    }

}
