package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Auto1Right")
public class Auto1Right extends LinearOpMode {

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
            Vector2D startPos = bot.drive.TileCords( new Vector2D( 0.4, 1.5 ) );
            bot.smd.setPoseEstimate( new Pose2d( startPos.getX(), startPos.getY(), 0) );

            //Vector2D targetPos = new Vector2D(0, 0);
            if (!parked) {
                //if (tag == null) telemetry.addLine("Tag is NULL");

                //PLACE CONE
                bot.drive.MoveAlongPath(
                        true,
                        new ArrayList<Double>(List.of(0.035, 0.5)),
                        bot.smd,
                        bot.telemetry,
                        this
                );
                bot.arm.AutoLiftToPos(1);
                bot.drive.MoveAlongPath(true, new ArrayList<Double>(List.of(0.12)), bot.smd, bot.telemetry, this);
                sleep(500);
                //region Gripper
                telemetry.addLine("Dropping cone");
                telemetry.update();

                //bot.arm.RotateGripperForDuration(false, 750);
                bot.arm.gripServo.setPower(1);
                telemetry.addLine("Dropping Commenced");
                telemetry.update();

                sleep(1500);

                telemetry.addLine("Dropping finished");
                telemetry.update();

                bot.arm.gripServo.setPower(0);
                //endregion
                sleep(500);
                bot.drive.MoveAlongPath(true, new ArrayList<Double>(List.of(-0.13)), bot.smd, bot.telemetry, this );
                bot.arm.AutoLiftToPos(0);

                //MOVE TO ZONE
                if (tag == null || tag.id == LEFT) {
                    //GO TO ZONE 1
                    bot.drive.MoveToPos(
                 false,
                            bot.drive.TileCords(new Vector2D(1.75, 2.5)),
                            bot.smd,
                            bot.telemetry,
                            this
                    );
                } else if (tag.id == MIDDLE) {
                    //GO TO ZONE 2
                    bot.drive.MoveToPos(
                            false,
                            bot.drive.TileCords(new Vector2D(1.75, 1.5)),
                            bot.smd,
                            bot.telemetry,
                            this
                    );
                } else {
                    //GO TO ZONE 3
                    bot.drive.MoveToPos(
                            false,
                            bot.drive.TileCords(new Vector2D(1.75, 0.5)),
                            bot.smd,
                            bot.telemetry,
                            this
                    );
                }

                parked = true;
            }
        }
    }
}