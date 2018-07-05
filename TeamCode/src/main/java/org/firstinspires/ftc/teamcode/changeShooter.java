package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.Arrays;
import java.util.Locale;


// * Created by definitly not HIRSH as he would mess it up and it would explode on 8/18/2016.
@Disabled
@TeleOp(name= "Change Shooter Position Thing")
public class changeShooter extends OpMode {

    DcMotor shooter;


    public changeShooter(){

    }
    public void init(){


        shooter = hardwareMap.dcMotor.get("shooter");
    }

    public void loop() {
        if(gamepad2.right_trigger == 1 || gamepad2.left_trigger == 1) {
            shooter.setPower(.1);
        }
        else {
            shooter.setPower(0);
        }
    }
}