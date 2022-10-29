package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.robot.*;
@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        Robot bot = new Robot(hardwareMap, telemetry);

        //Tag IDs for 3 different park locations
        int LEFT = 11;
        int MIDDLE = 12;
        int RIGHT = 13;

        /*AprilTagDetection tag = bot.camera.runTagDetection();

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

        sleep(5000);*/

        waitForStart();
/*==================================================================================================
////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
||||||||||||||||||||||||||||||||||||||||| BEGIN OPMODE |||||||||||||||||||||||||||||||||||||||||||||
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////////////////
==================================================================================================*/

        while (opModeIsActive()) {
            //do stuff

            telemetry.addLine("Starting OpMode");
            telemetry.update();

//            sleep(3000);

            bot.arm.gripTest();
        }
    }

}