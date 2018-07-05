package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Luke on 11/25/2016.
 */
@Disabled
@Autonomous(name = "Speed thing thats annoying", group = "HDrive")
public class SpeedTest extends LinearOpMode {
    DcMotor middleMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        middleMotor = hardwareMap.dcMotor.get("middleMotor");

        waitForStart();
        middleMotor.setPower(.2);
        Thread.sleep(40000);
        middleMotor.setPower(0);


    }
}
