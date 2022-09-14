package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveBase {
    public double speed = 1.0;

    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftRearDrive;
    public DcMotor rightRearDrive;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public DriveBase (HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "LRDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "RRDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.telemetry = telemetry;
    }

    public double getGyroAngle() {
        return angles.firstAngle;
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
        leftFrontDrive.setPower(-1 * lf);
        leftRearDrive.setPower(-1 * lr);
        rightFrontDrive.setPower(rf);
        rightRearDrive.setPower(rr);
    }

    public void driveForward(double speed) {
        this.speed = speed;
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftRearDrive.setPower(speed);
        rightRearDrive.setPower(speed);
    }
    public void driveRight(double speed) {
        this.speed = speed;
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftRearDrive.setPower(-speed);
        rightRearDrive.setPower(speed);
    }
    public void turnRight(double speed) {
        this.speed = speed;
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftRearDrive.setPower(speed);
        rightRearDrive.setPower(-speed);
    }

}
