package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Camera;

public class Robot {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public Camera camera;
    public DriveBase drive; //Remove drive because smd?
    public Arm arm;

    public SampleMecanumDrive smd;

    public InputManager inputMan;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new DriveBase(hardwareMap, telemetry);
        camera = new Camera(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);

        smd = new SampleMecanumDrive(hardwareMap);

        inputMan = new InputManager(this);
    }
}
