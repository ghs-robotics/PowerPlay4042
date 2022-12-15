package org.firstinspires.ftc.teamcode.odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.odometry.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class OdometryDrive {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 17.5 / 25.4; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.25; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 46.25 / 25.4; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.0507679979166666666666666666667;
    public static double Y_MULTIPLIER = 1.0507679979166666666666666666667;


    private Encoder leftEncoder, rightEncoder, frontEncoder;

    List<LynxModule> allHubs;

    private boolean readMode = false;

    public OdometryDrive(HardwareMap hardwareMap){
        List<Pose2d> wheelPoses;// front
        wheelPoses = new ArrayList<Pose2d>();

        wheelPoses.add(new Pose2d(0, LATERAL_DISTANCE / 2, 0)); //left
        wheelPoses.add(new Pose2d(0, -LATERAL_DISTANCE / 2, 0));//right
        wheelPoses.add(new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)));//front




        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BLDrive")); //left
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BRDrive")); //right
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FLDrive"));

        allHubs = hardwareMap.getAll(LynxModule.class);

        //TODO need to change it depending on new drivebase
        leftEncoder.setDirection(Encoder.Direction.FORWARD);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
    }

}
