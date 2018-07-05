package org.firstinspires.ftc.teamcode;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.hardware.GyroSensor;
        import com.qualcomm.robotcore.hardware.AnalogInput;
        import com.qualcomm.robotcore.hardware.AnalogInputController;
/**
 * Created by Luke on 9/21/2016.
 */
public class AutoClass extends OpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotorController controller;
    public void init(){
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
        controller = hardwareMap.dcMotorController.get("Motor Controller 1");

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void loop(){
        telemetry.addData("Motor 1 position", controller.getMotorCurrentPosition(1));
        telemetry.addData("Motor 2 position", controller.getMotorCurrentPosition(2));
        leftMotor.setTargetPosition(3000);
        rightMotor.setTargetPosition(3000);

    }

}
