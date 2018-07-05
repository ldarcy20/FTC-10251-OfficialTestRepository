package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@Autonomous(name = "Sensor: Touch Sensor", group = "HDrive") // Comment this out to add to the opmode list
public class TouchSensorTest extends LinearOpMode {


    TouchSensor touchsensor;

    @Override
    public void runOpMode() throws InterruptedException {



        touchsensor = hardwareMap.touchSensor.get("touch_sensor");

        waitForStart();


        // idle until the touch sensor is pressed
        while (!touchsensor.isPressed()) {
            telemetry.addData("isPressed",String.valueOf(touchsensor.isPressed()));
            telemetry.update();
        }



        telemetry.addData("Touch Sensor was Pressed",String.valueOf(touchsensor.isPressed()));
        telemetry.update();
    }
}