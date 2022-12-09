package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimitSwitch {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DigitalChannel digInput;

    public LimitSwitch(Robot robot){
        try {
            telemetry = robot.telemetry;
            hardwareMap = robot.hardwareMap;
            digInput = hardwareMap.get(DigitalChannel.class, "liftLim");
        }
        catch (Exception e) {
            telemetry.addData("Limit switch doesn't work", "fail");
        }
    }

        public boolean isPressed (){
            return digInput.getState();
        }
}
