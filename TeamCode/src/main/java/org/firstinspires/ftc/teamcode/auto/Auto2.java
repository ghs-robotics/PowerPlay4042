//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
//import org.firstinspires.ftc.teamcode.robot.Robot;
//
//@Autonomous(name = "Auto2")
//public class Auto2 extends LinearOpMode {
//
//    private boolean parked = false;
//    private int color;
//    /**
//     * This function is executed when this Op Mode is selected from the Driver Station.
//     */
//    @Override
//    public void runOpMode() {
//        Robot bot = new Robot(hardwareMap, telemetry);
//
//        //Tag IDs for 3 different park locations
//        int LEFT = 0; //Lime
//        int MIDDLE = 1; //Magenta
//        int RIGHT = 2; //Cyan
//
//        bot.camera.setupColorDetection();
//
//        while(!opModeIsActive()) {
//            color = bot.camera.runColorDetection();
//        }
//
///*==================================================================================================
//////////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//||||||||||||||||||||||||||||||||||||||||| BEGIN OPMODE |||||||||||||||||||||||||||||||||||||||||||||
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\////////////////////////////////////////////////////
//==================================================================================================*/
//
//
//        while (opModeIsActive()) {
//            //SET START POS
//            Vector2D startPos = bot.drive.TileCords( new Vector2D( 0.5, 4.5 ) );
//            bot.smd.setPoseEstimate( new Pose2d( startPos.getX(), startPos.getY(), 0) );
//
//            Vector2D targetPos = new Vector2D(0, 0);
//
//            if (color == -1) telemetry.addLine("Color is -1");
//
//            if (color == LEFT) {
//                //GO TO ZONE 1
//                targetPos = bot.drive.RelativeToGlobalPos( new Vector2D( 1.4f, 1 ), bot.smd );
//                telemetry.addLine("Setting path to Zone 1");
//            } else if (color == -1 || color == MIDDLE) {
//                //GO TO ZONE 2
//                targetPos = bot.drive.RelativeToGlobalPos( new Vector2D( 1.4f, 0 ), bot.smd );
//                telemetry.addLine("Setting path to Zone 2");
//            } else {
//                //GO TO ZONE 3
//                targetPos = bot.drive.RelativeToGlobalPos( new Vector2D( 1.4f, -1 ), bot.smd );
//                telemetry.addLine("Setting path to Zone 3");
//            }
//
//            //MOVE TO TARGET POSITION
//            telemetry.addLine("Moving to Zone");
//            telemetry.update();
//            if (!parked) {
//                bot.drive.MoveToPos(
//                        true,
//                        bot.drive.RelativeToGlobalPos( new Vector2D(0.1f, 0), bot.smd),
//                        bot.smd,
//                        bot.telemetry
//                );
//
//                bot.drive.MoveToPos( false, targetPos, bot.smd, telemetry );
//                parked = true;
//            }
//
//            telemetry.addLine("Ending OPMode");
//            telemetry.update();
//
//        }
//    }
//
//}