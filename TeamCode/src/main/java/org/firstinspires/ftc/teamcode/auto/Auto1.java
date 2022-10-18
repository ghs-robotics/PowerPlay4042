package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.DetectionSystem;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.robot.*;
@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Robot drive = new Robot(hardwareMap, telemetry);

        //Tag IDs for 3 different park locations
        int LEFT = 11;
        int MIDDLE = 12;
        int RIGHT = 13;
        AprilTagDetection tag = DetectionSystem.runTagDetection(hardwareMap, telemetry);

        if(tag == null || tag.id == LEFT) {
            //Go to left parking spot
            telemetry.addLine("Pathing to left parking spot");
            telemetry.update();
        } else if(tag.id == MIDDLE) {
            //Go to middle parking spot
            telemetry.addLine("Pathing to middle parking spot");
            telemetry.update();
        } else {
            //Go to right parking spot
            telemetry.addLine("Pathing to right parking spot");
            telemetry.update();
        }

        sleep(5000);
/*==================================================================================================
////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
||||||||||||||||||||||||||||||||||||||||| BEGIN OPMODE |||||||||||||||||||||||||||||||||||||||||||||
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////////////////
==================================================================================================*/

        if (opModeIsActive()) {
            sleep(20);
            //do stuff
        }
    }

}