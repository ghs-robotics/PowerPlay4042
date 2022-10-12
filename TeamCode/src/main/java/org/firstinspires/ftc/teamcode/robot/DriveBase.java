package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.odometry.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.odometry.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveBase extends MecanumDrive {
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    HardwareMap hardwareMap;

    private DcMotorEx leftFrontDrive;
    private DcMotorEx leftRearDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx rightRearDrive;
    private List<DcMotorEx> motors;

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;


    private TrajectorySequenceRunner trajectorySequenceRunner;

    private TrajectoryFollower follower;

    private VoltageSensor batteryVoltageSensor;

    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.hardwareMap = hardwareMap;

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFrontDrive = hardwareMap.get(DcMotorEx .class,"LFDrive");
        leftRearDrive = hardwareMap.get(DcMotorEx .class,"LRDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx .class,"RFDrive");
        rightRearDrive = hardwareMap.get(DcMotorEx .class,"RRDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors = Arrays.asList(leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive);

        setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

    }

    public void calculateDrivePower(double x, double y, double r){
        r = -r;
        double lf = r - x + y;
        double lr = r + x + y;
        double rf = r - x - y;
        double rr = r + x - y;

        setMotorPowers(rf, rr, lf, lr);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public void update() {
        updatePoseEstimate();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double rf, double rr, double lf, double lr) {
        leftFrontDrive.setPower(-lf);
        leftRearDrive.setPower(-lr);
        rightFrontDrive.setPower(rf);
        rightRearDrive.setPower(rr);
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }
}
