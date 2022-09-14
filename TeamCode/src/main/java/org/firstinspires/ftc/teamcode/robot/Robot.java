package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DcMotor intakeMotor;
    public DcMotor shooterMotor;
    public DcMotor extendingLiftMotor;

    public Servo indexingServo;
    public Servo liftingServo;

    public static void calculateDrivePower(float left_stick_x, float left_stick_y, float right_stick_x, float right_stick_y) {
    }

    //we should rename these

}
