package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Auto1")
public class Auto1 extends LinearOpMode {

    private boolean parked = false;
    private AprilTagDetection tag;
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

        bot.camera.setupTagDetection();

        while(!opModeIsActive()) {
          tag = bot.camera.runTagDetection();
        }

/*==================================================================================================
////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
||||||||||||||||||||||||||||||||||||||||| BEGIN OPMODE |||||||||||||||||||||||||||||||||||||||||||||
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////////////////
==================================================================================================*/


        while (opModeIsActive()) {
            //SET START POS
            Vector2D startPos = bot.autoMove.TileCords( new Vector2D( 0.5, 4.5 ) );
            bot.smd.setPoseEstimate( new Pose2d( startPos.getX(), startPos.getY(), 0) );

            //Vector2D targetPos = new Vector2D(0, 0);
            if (!parked) {
                if (tag == null) telemetry.addLine("Tag is NULL");

                if (tag == null || tag.id == LEFT) {
                    //GO TO ZONE 1
                    bot.autoMove.MoveAlongPath(
                            false,
                            new ArrayList<Double>(List.of(1.0, 1.5)),
                            bot.smd,
                            bot.telemetry
                    );

                    //targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1.5f, 0 ), bot.smd );
                    //telemetry.addLine("Setting path to Zone 1: pos:" + targetPos.toString());
                } else if (tag.id == MIDDLE) {
                    //GO TO ZONE 2
                    bot.autoMove.MoveAlongPath(
                            true,
                            new ArrayList<Double>(List.of(1.5)),
                            bot.smd,
                            bot.telemetry
                    );

                    //targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1.5f, 0 ), bot.smd );
                    //telemetry.addLine("Setting path to Zone 2: pos:" + targetPos.toString());
                } else {
                    //GO TO ZONE 3
                    bot.autoMove.MoveAlongPath(
                            false,
                            new ArrayList<Double>(List.of(-1.0, 1.5)),
                            bot.smd,
                            bot.telemetry
                    );

                    //targetPos = bot.autoMove.RelativeToGlobalPos( new Vector2D( 1.5f, -1 ), bot.smd );
                    //telemetry.addLine("Setting path to Zone 3: pos:" + targetPos.toString());
                }

                parked = true;
            }

            //MOVE TO TARGET POSITION
            /*telemetry.addLine("Moving to Zone: pos:" + targetPos.toString());
            telemetry.update();
            if (!parked) {
                bot.autoMove.MoveToPosOld( targetPos, bot.smd, telemetry );
                parked = true;
            }*/

            telemetry.addLine("Ending OPMode");
            telemetry.update();

        }
    }

}