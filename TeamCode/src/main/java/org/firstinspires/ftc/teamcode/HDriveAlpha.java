package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
@TeleOp(name= "TeleOp Alpha")
public class HDriveAlpha extends OpMode {
    double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    ColorSensor sensorRGB;
    Orientation angles;
    BNO055IMU imu;
    String angleDouble = "hi";
    public double gyroAngle;
    boolean speedMode = false;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotor shooter;
    boolean countUp = false;
    int countsinceapressed = 0;
    boolean countUp2 = false;
    int countsincebpressed = 0;
    AnalogInput distanceSensor;
    HDriveFCCalc calculator;
    Servo servo2;
    double armAngle = .5;
    double offset = 0;
    int encoder = 0;
    int shootTimer = 6;
    boolean bumperPressed = false;
    boolean bumperIsPressed = false;
    boolean hulianNotAnH = false;
    boolean shootTimerDone = false;
    Servo buttonPusher;
    double start;
    boolean startingButton = false;
    boolean state1 = false; boolean state2 = false; boolean state3 = false; boolean state4 = false; boolean state5 = false; boolean state6 = false; boolean state7 = false; boolean state8 = false; boolean state9 = false; boolean state10 = false;
    double time1; double time2 = 3001; double time3; double time4; double time5; double time6; double time7;
    boolean running1 = false; boolean running2 = false; boolean running3 = false; boolean running4 = false; boolean running5 = false; boolean running6 = false; boolean running7 = false; boolean running8 = false;
    boolean starting[] = new boolean[10];
    boolean beaconRed = false;
    boolean beaconBlue = false;
    public HDriveAlpha(){
        start = System.currentTimeMillis();
    }
    public void init(){
        Arrays.fill(starting, false);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        middleMotor = hardwareMap.dcMotor.get("middleMotor");
        shooter = hardwareMap.dcMotor.get("shooter");
        distanceSensor = hardwareMap.analogInput.get("ODS");
        buttonPusher = hardwareMap.servo.get("servo2");
        sensorRGB = hardwareMap.colorSensor.get("color");



        calculator = new HDriveFCCalc();
        //servo = hardwareMap.crservo.get("servo");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        int i = 0;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angleDouble = formatAngle(angles.angleUnit, angles.firstAngle);
        float leftX = gamepad1.left_stick_x;
        float leftY = gamepad1.left_stick_y;
        float rightX = gamepad1.right_stick_x;
        float rightY = gamepad1.right_stick_y;
        float left = gamepad1.left_trigger;
        float right = gamepad1.right_trigger;
        float leftTrigger = gamepad2.left_trigger;
        float rightTrigger = gamepad2.right_trigger;
        boolean buttonAPressed = gamepad1.a;
        boolean buttonXPressed = gamepad1.x;
        boolean buttonAPressed2 = gamepad2.a;
        boolean buttonXPressed2 = gamepad2.x;
        //bumperPressed = gamepad1.right_bumper;
        if (countUp) {
            if (countsinceapressed < 10) {
                countsinceapressed++;
            } else {
                countUp = false;
                countsinceapressed = 0;
            }
        }
        if (buttonAPressed && !countUp) {
            countUp = true;
            if (speedMode == false) {
                speedMode = true;
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //try.addData("Mode", "Speed");
               // telemetry.update();
            } else if (speedMode == true) {
                speedMode = false;
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //telemetry.addData("Mode", "Power");
                //telemetry.update();
            }
        }
        // telemetry.addData("Encoder Position", shooter.getCurrentPosition());
        // telemetry.addData("Angle", Double.parseDouble(angleDouble)+offset);
        // telemetry.addData("Left Trigger", gamepad1.left_trigger);
        // telemetry.addData("Left Stick X" , gamepad1.left_stick_x);
        //telemetry.addData("Right Stick Y" , gamepad1.right_stick_x);
        //telemetry.addData("Left Stick X" , gamepad1.left_stick_y);
        //telemetry.addData("Right Stick Y" , gamepad1.right_stick_y);
        //telemetry.update();


        if (buttonXPressed == true) {
            offset = Double.parseDouble(angleDouble);
            offset = -offset;
        }


        calculator.calculateMovement(leftX, leftY, rightX, Double.parseDouble(angleDouble) + offset);
        if (gamepad1.right_bumper == false) {
            if (!speedMode) {
                leftMotor.setPower(.7 * calculator.getLeftDrive());
                rightMotor.setPower(.7 * calculator.getRightDrive());
                middleMotor.setPower(-calculator.getMiddleDrive());
            } else {
                leftMotor.setPower(calculator.getLeftDrive());
                rightMotor.setPower(calculator.getRightDrive());
                middleMotor.setPower(-calculator.getMiddleDrive());
            }
        }
        if (left > 0) {
            buttonPusher.setPosition(1);
        }
        if (right > 0) {
            buttonPusher.setPosition(0);
        }
     /*   if(gamepad1.left_bumper == true) {
            leftMotor.setPower(-.1);
            rightMotor.setPower(-.1);
        }
        if(gamepad1.right_bumper == true) {
            leftMotor.setPower(.1);
            rightMotor.setPower(.1);
<<<<<<< HEAD
        }*/
        if (!shootTimerDone) {
            shootTimer++;
            if (shootTimer > 5) {
                shootTimerDone = true;
            }
        }
        if (gamepad1.left_bumper && shootTimerDone && !bumperIsPressed) {
            shooter.setTargetPosition(3360);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setPower(.3);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bumperIsPressed = true;
            //telemetry.addLine(Double.toString(shooter.getCurrentPosition()));
            //telemetry.update();
            shootTimer = 0;
        }
        if (shooter.getCurrentPosition() >= 3359) {
            shootTimerDone = false;
            shooter.setPower(0);
            bumperIsPressed = false;
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // telemetry.addLine(Double.toString(shooter.getCurrentPosition()));
           // telemetry.update();
        }
       // telemetry.addLine(Double.toString(shooter.getCurrentPosition()));
        //telemetry.update();
        /*if(gamepad1.right_bumper) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setTargetPosition(3120);
            shooter.setPower(1);
            shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hulianNotAnH = true;
        }
        //set to maximum overdrive
        if(shooter.getCurrentPosition() == 3120 && hulianNotAnH == true) {
            shooter.setPower(0);
            hulianNotAnH = false;
            shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        servo2.setPosition(armAngle);
        */
        //**********************************************************************
        //telemetry.addData("Time", System.currentTimeMillis()-start);
        //telemetry.addData("Distance Sesnor", distanceSensor.getVoltage());
        //telemetry.addData("Color Sensor", sensorRGB.red());
        //telemetry.addData("Shooter", shooter.getMode());
        //telemetry.addData("Shooter", shooter.getCurrentPosition());
        //telemetry.addData("Bumper", gamepad1.left_bumper);
        //telemetry.update();
        if (gamepad1.right_bumper) {
            middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(starting[1]) {
                time2 = System.currentTimeMillis() - time1;
            }
            if(starting[0]) {
                time4 = System.currentTimeMillis() - time3;
            }
            if(starting[4]) {
                time6 = System.currentTimeMillis() - time5;
            }
            if (gamepad1.right_bumper && time2 > 3000) {
                bumperPressed = true;
                startingButton = true;
                time2 = System.currentTimeMillis() - time1;
                starting[1] = true;
            } else {
                bumperPressed = false;
            }
            if (startingButton) { //check if presed only once.
                startingButton = false;
                state1 = true;
                time1 = System.currentTimeMillis();
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (state1) {
                leftMotor.setPower(.1);
                rightMotor.setPower(.1);
                middleMotor.setPower(.1);
                if(distanceSensor.getVoltage() > .7)  {
                    state1 = false;
                    state2 = true;
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    middleMotor.setPower(0);
                }
            }
            if(state2) { //delay after finding beacon.
                running1 = true;
            }
            if(running1) {//delay after finding beacon, only does this once
                running1 = false;
                buttonPusher.setPosition(0);
                time3 = System.currentTimeMillis();
                starting[0] = true; //tells it to start checking the time for 100 ms delay
                state2 = false;
                state3 = true;
            }
            if(state3 && time4 > 100) {
                starting[0] = false;
                if(sensorRGB.red() < sensorRGB.blue()) {
                    beaconBlue = true;
                    state3 = false;
                    starting[2] = true;
                    telemetry.addData("Beacon State", sensorRGB.blue());
                    telemetry.update();
                }
                if(sensorRGB.blue() < sensorRGB.red()) {
                    beaconRed = true;
                    state3 = false;
                    starting[2] = true;
                    telemetry.addData("Beacon State", sensorRGB.red());
                    telemetry.update();
                }
            }
            if(starting[2]) {
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                starting[2] = false;
            }
            if(beaconBlue) {
                leftMotor.setTargetPosition(83);
                rightMotor.setTargetPosition(83);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setPower(.3);
                rightMotor.setPower(.3);
                beaconBlue = false;
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                state5 = true;
            }
            if(leftMotor.getCurrentPosition() >= 83 && state5) {
                telemetry.addLine("Got There");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                state5 = false;
                state6 = true;
                beaconBlue = false;
                buttonPusher.setPosition(1);
            }
            if(state6) {
                telemetry.addLine("Got there again good job");
                buttonPusher.setPosition(1);
                state5 = false;
                starting[3] = true;
            }
            if(starting[3]) {
                buttonPusher.setPosition(1);
                starting[3] = false;
                time5 = System.currentTimeMillis();
                state6 = false;
                state7 = true;
                starting[4] = true;
            }
            if(state7 && time6 > 500) {
                starting[4] = false;
                buttonPusher.setPosition(0);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }




            if(beaconRed) {
                leftMotor.setTargetPosition(440);
                rightMotor.setTargetPosition(440);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setPower(.4);
                rightMotor.setPower(.4);
                beaconRed = false;
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                state8 = true;
            }
            if(leftMotor.getCurrentPosition() >= 439 && state8) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                state8 = false;
                state9 = true;
                beaconRed = false;
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(state9) {
                buttonPusher.setPosition(1);
                state8 = false;
                starting[3] = true;
            }
            if(starting[3]) {
                starting[3] = false;
                time5 = System.currentTimeMillis();
                state9 = false;
                state10 = true;
                starting[4] = true;
            }
            if(state10 && time6 > 500) {
                starting[4] = false;
                buttonPusher.setPosition(0);
            }

            //***********************************************************************/
        }
        else {
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            state1 = false;
            state2 = false;
            state3 = false;
            state4 = false;
            state5 = false;
            state6 = false;
            state7 = false;
            state8 = false;
            state9 = false;
            state10= false;
            time1 = 0; time2 = 3001; time3 = 0; time4 = 0; time5 = 0; time6 = 0; time7 = 0;
            running1 = false; running2 = false; running3 = false; running4 = false; running5 = false; running6 = false; running7 = false; running8 = false;
            Arrays.fill(starting, false);
            beaconBlue = false;
            beaconRed  = false;
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));


    }
    public double elapsedTime() {
        long now = System.currentTimeMillis();
        return (now - start);
    }

}
