package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotor liftMotor;
    private DcMotor gripperMotor;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        liftMotor = hardwareMap.get(DcMotor.class,"LiftMot");
        gripperMotor = hardwareMap.get(DcMotor.class, "GripMot");
    }

    public void moveArm() {
        liftMotor.setPower(10);
    }

    public void grip() {
        gripperMotor.setPower(10);
    }


}
