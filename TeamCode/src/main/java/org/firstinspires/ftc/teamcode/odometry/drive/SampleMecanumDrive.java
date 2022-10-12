package org.firstinspires.ftc.teamcode.odometry.drive;

import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.odometry.drive.DriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.odometry.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.odometry.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.odometry.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {

    pose

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private VoltageSensor batteryVoltageSensor;

    //region AutonomousMovement -------------------------------------------------------------------------------------------|
    private final Vector2D ArenaDimensions = new Vector2D( 144, 144 );
    private final Vector2D TileDimensions = new Vector2D( 24, 24 );
    private final Vector2D TileNumber =
            new Vector2D(
                    ArenaDimensions.getX() / TileDimensions.getX(),
                    ArenaDimensions.getY() / TileDimensions.getY()
            );

    private final double StopDistThreshold = 0.1;

    public Vector2D TileCords( Vector2D index, Vector2D percentInTile ) {
        //index from ( 0, 0 ) to ( 5, 5 )
        Vector2D bottomLeftCorner = new Vector2D(
                ( index.getX() * TileDimensions.getX() ) - ( ArenaDimensions.getX() / 2 ),
                ( index.getY() * TileDimensions.getY() ) - ( ArenaDimensions.getY() / 2 )
        );
        Vector2D distInTile = new Vector2D(
                percentInTile.getX() * TileDimensions.getX(),
                percentInTile.getY() * TileDimensions.getY()
        );
        return new Vector2D(
                bottomLeftCorner.getX() + distInTile.getX(),
                bottomLeftCorner.getY() + distInTile.getY()
        );
    }
    public void MoveToPos( Vector2D target ) {
        Pose2d crntPos = getPoseEstimate();
        float spd = 1; //Could be modified throughout the function to smoothly stop and start.

        int axesMovedOn = 0;
        boolean moveOnXAxis = false;

        //Check if X axis of the robot is further from a poll
        if ( Math.abs( ( Math.abs( crntPos.getX() ) % TileDimensions.getX() ) - ( TileDimensions.getX() / 2 ) ) <
                Math.abs( ( Math.abs( crntPos.getY() ) % TileDimensions.getY() ) - ( TileDimensions.getY() / 2 ) ) ) {
            moveOnXAxis = true;
        }

        while ( axesMovedOn < 2) {
            crntPos = new Pose2d( 0, 0, 0 );

            if ( moveOnXAxis ) {
                double dif = target.getX() - crntPos.getX();
                if ( dif > StopDistThreshold ) {
                    setWeightedDrivePower( new Pose2d( Math.signum( dif ) * spd, 0, 0 ) );
                }
                else {
                    moveOnXAxis = false;
                    axesMovedOn++;
                }
            }
            else {
                double dif = target.getY() - crntPos.getY();
                if ( dif > StopDistThreshold ) {
                    setWeightedDrivePower( new Pose2d( 0, Math.signum( dif ) * spd, 0 ) );
                }
                else {
                    moveOnXAxis = true;
                    axesMovedOn++;
                }
            }
        }
    }
    //endregion -----------------------------------------------------------------------------------------------------------|

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "LFDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "LRDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "RRDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "RFDrive");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse any motors using DcMotor.setDirection()

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
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
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        rightFront.setPower(v);
        rightRear.setPower(v1);
        leftRear.setPower(-v2);
        leftFront.setPower(-v3);
//
//        leftFront.setPower(v1);
//        leftRear.setPower(v);
//        rightRear.setPower(v3);
//        rightFront.setPower(v2);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
