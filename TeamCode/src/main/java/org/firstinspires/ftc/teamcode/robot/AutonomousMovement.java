package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonomousMovement {
    private final Vector2D TileDimensions = new Vector2D(23.5f, 23.5f);
    private final Vector2D TileNumber = new Vector2D(6.0, 6.0);
    private final Vector2D ArenaDimensions =
        new Vector2D(
            TileDimensions.getX() * TileNumber.getX(),
            TileDimensions.getY() * TileNumber.getY()
        );

    private final double MoveToSpd = 0.5;
    private final double MoveToSlowDist = 6;
    private final double MoveToSlowSpd = 0.25;
    private final double MoveToStopDist = 0.15;

    private final double ArmStopDist = 0.1;

    public void LiftToPos( int targetPos, Arm arm, Telemetry telemetry ) {
        DcMotorEx liftMotor1 = arm.liftMotor1;
        DcMotorEx liftMotor2 = arm.liftMotor2;

        int crntPos = liftMotor1.getCurrentPosition();
        int error = liftMotor2.getCurrentPosition() - arm.liftMotor2.getCurrentPosition();//add to telemetry?

        double dif = targetPos - crntPos;
        double absDif = Math.abs( dif );

        liftMotor1.setTargetPosition(targetPos);
        liftMotor2.setTargetPosition(targetPos); //account for error?

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.driveArm(1);//does this stay set or need called every frame

        while (absDif > ArmStopDist) {
            crntPos = liftMotor1.getCurrentPosition();

            dif = targetPos - crntPos;
            absDif = Math.abs( dif );
        }

        arm.driveArm(0);

        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
