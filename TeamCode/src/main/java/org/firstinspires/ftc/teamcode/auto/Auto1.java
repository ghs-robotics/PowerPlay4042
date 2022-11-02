package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.openftc.apriltag.AprilTagDetection;
@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {

    private boolean parked = false;
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

        AprilTagDetection tag = bot.camera.runTagDetection();

        sleep(5000);

        waitForStart();
/*==================================================================================================
////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
||||||||||||||||||||||||||||||||||||||||| BEGIN OPMODE |||||||||||||||||||||||||||||||||||||||||||||
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////////////////
==================================================================================================*/


        while (opModeIsActive()) {
            Vector2D startPos = bot.autoMove.TileCords( new Vector2D( 0.5, 4.5 ) );
            bot.smd.setPoseEstimate( new Pose2d( startPos.getX(), startPos.getY(), 0) );

            Vector2D targetPos = new Vector2D(0, 0);

            if (tag == null) telemetry.addLine("Tag is NULL");

            if (tag == null || tag.id == LEFT) {
                //Go to Zone 1
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1.5f, 1 ), bot.smd );
                telemetry.addLine("Setting path to Zone 1");
            } else if (tag.id == MIDDLE) {
                //Go to Zone 2
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1.5f, 0 ), bot.smd );
                telemetry.addLine("Setting path to Zone 2");
            } else {
                //Go to Zone 3
                targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1.5f, -1 ), bot.smd );
                telemetry.addLine("Setting path to Zone 3");
            }

            telemetry.addLine("Moving to Zone");
            telemetry.update();
            if ( parked == false ) {
                bot.autoMove.MoveToPos( targetPos, bot.smd, telemetry );
                parked = true;
            }

            telemetry.addLine("Ending OPMode");
            telemetry.update();

            break;
        }
    }

}