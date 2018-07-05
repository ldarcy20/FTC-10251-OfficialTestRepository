package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Disabled
@TeleOp(name= "Optical Distance Sensor", group = "HDrive")
public class AnalogInputTest extends OpMode {


        OpticalDistanceSensor opticalDistanceSensor;
        AnalogInput distanceSensor;
        @Override
        public void init() {
                distanceSensor = hardwareMap.analogInput.get("ODS");
        }


        @Override
        public void loop() {
                //double value = distanceSensor.getLightDetected();


                telemetry.addData("Value", distanceSensor.getVoltage());


        }
}




