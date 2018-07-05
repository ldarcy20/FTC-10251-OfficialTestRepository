package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@Disabled
@TeleOp(name= "Tinker Bell", group = "HDrive")
public class TinkerBell extends OpMode {


    OpticalDistanceSensor opticalDistanceSensor;
    AnalogInput tinkerBellAnalog;
    DigitalChannel tinkerBellDigital;

    @Override
    public void init() {
        tinkerBellAnalog = hardwareMap.analogInput.get("ODS");
        tinkerBellDigital = hardwareMap.digitalChannel.get("DigitalInput");
    }


    @Override
    public void loop() {
        //double value = distanceSensor.getLightDetected();


        telemetry.addData("Value", tinkerBellAnalog.getVoltage());
        telemetry.addData("DigitalInput", tinkerBellDigital.getState());


    }
}




