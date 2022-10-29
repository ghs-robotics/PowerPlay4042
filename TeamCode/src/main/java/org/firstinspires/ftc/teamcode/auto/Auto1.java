package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.odometry.drive.SampleMecanumDrive;
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Tag IDs for 3 different park locations
        int LEFT = 11;
        int MIDDLE = 12;
        int RIGHT = 13;


        AprilTagDetection tag = bot.camera.runTagDetection();


        sleep(5000);

        waitForStart();
/*==================================================================================================
////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
||||||||||||||||||||||||||||||||||||||||| BEGIN OPMODE |||||||||||||||||||||||||||||||||||||||||||||
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////////////////
==================================================================================================*/


        while (opModeIsActive()) {
            //do stuff

            Vector2D startPos = bot.autoMove.TileCords( new Vector2D( 0, 5 ), new Vector2D( 0.5, 0.5 ) );
            bot.smd.setPoseEstimate( new Pose2d( startPos.getX(), startPos.getY(), 0) );
            Vector2D targetPos = new Vector2D(0, 0);
            if(tag == null || tag.id == LEFT) {
                //Go to left parking spot
                targetPos = bot.autoMove.TileCords( new Vector2D( 1, 6 ), new Vector2D( 1, 0.5 ) );
                telemetry.addLine("Pathing to left parking spot");
                telemetry.update();
            } else if(tag.id == MIDDLE) {
                targetPos = bot.autoMove.TileCords( new Vector2D( 1, 5 ), new Vector2D( 1, 0.5 ) );
                //Go to middle parking spot
                telemetry.addLine("Pathing to middle parking spot");
                telemetry.update();
            } else {
                targetPos = bot.autoMove.TileCords( new Vector2D( 1, 4 ), new Vector2D( 1, 0.5 ) );
                //Go to right parking spot
                telemetry.addLine("Pathing to right parking spot");
                telemetry.update();
            }
            bot.autoMove.MoveToPosLoop( targetPos, bot.smd, telemetry );

            telemetry.addLine("Starting OpMode");
            telemetry.update();

            bot.arm.gripTest();
        }
    }

}