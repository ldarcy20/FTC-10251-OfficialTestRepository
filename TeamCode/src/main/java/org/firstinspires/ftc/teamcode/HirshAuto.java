package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by Hirsh on 9/22/2016.
 */
public class HirshAuto extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;

    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
    }

    public void loop() {
        leftMotor.setTargetPosition(3000);
        rightMotor.setTargetPosition(3000);
        middleMotor.setTargetPosition(3000);

    } //Hirshes computer is bad -Lukes computer which doesn't know grammar 2K16

}
