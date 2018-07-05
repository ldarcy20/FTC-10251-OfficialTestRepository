package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
/**
 * Created by Luke on 12/4/2016.
 */
@Disabled
@TeleOp(name = "Digital Input yas")
public class DigitalInput extends OpMode{
    DigitalChannel input;
    @Override
    public void init() {
        input = hardwareMap.digitalChannel.get("DigitalInput");
    }

    @Override
    public void loop() {
        telemetry.addData("DigitalInput", input.getState());
        telemetry.update();
    }
}
