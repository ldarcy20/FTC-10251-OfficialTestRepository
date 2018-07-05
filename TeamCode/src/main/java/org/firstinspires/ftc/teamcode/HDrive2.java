package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;


// * Created by definitly not HIRSH as he would mess it up and it would explode on 8/18/2016.

@TeleOp(name= "Right Not Field Centric")
public class HDrive2 extends OpMode {
    double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];
    Orientation angles;
    BNO055IMU imu;
    String angleDouble = "hi";
    public double gyroAngle;
    boolean speedMode = false;
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotorEx middleMotor;
    DcMotorEx hirshIsDumb; //Pulley
    DcMotorEx arm;
    Servo servo;
    Servo servo3;
    Servo claw1;
    Servo claw2;
    boolean countUp = false;
    int countsinceapressed = 0;
    boolean countUp2 = false;
    int countsincebpressed = 0;
    //HDrive2 calculator;
    HDriveNonFeildCentric calculator;
    Servo servo2;
    double armAngle = .5;
    double offset = 0;
    int encoder = 0;
    int shootTimer = 6;
    boolean bumperPressed = false;
    boolean bumperIsPressed = false;
    boolean hulianNotAnH = false;
    boolean shootTimerDone = false;
    boolean stateGlyph = false;
    Servo buttonPusher;
    boolean lezGoSlow = false;
    boolean state1;
    boolean closedClaw = false;
    boolean runningclaw = false;
    int glyphCounter = 0;
    Servo IntakeServo;
    DcMotor IntakeMotor;
    public HDrive2(){

    }
    public void init(){

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
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftMotor");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "middleMotor");
        hirshIsDumb = (DcMotorEx)hardwareMap.get(DcMotor.class, "Hirsh is very dumb");
        claw1 = hardwareMap.servo.get("claw1");
        claw2 = hardwareMap.servo.get("claw2");
        arm = (DcMotorEx)hardwareMap.get(DcMotor.class, "arm");
        //shooter = hardwareMap.dcMotor.get("shooter");
        //servo3 = hardwareMap.servo.get("servo3");
        //arm = hardwareMap.dcMotor.get("arm");
        //servo2 = hardwareMap.servo.get("servo2");
        //buttonPusher = hardwareMap.servo.get("servo2");



        calculator = new HDriveNonFeildCentric();
        //servo = hardwareMap.crservo.get("servo");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.REVERSE);
        hirshIsDumb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hirshIsDumb.setDirection(DcMotor.Direction.REVERSE);
        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){
        if(glyphCounter < 10){
            glyphCounter++;
        }
        int i = 0;
        angles   = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
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
        boolean dPadUp    = gamepad1.dpad_up;
        boolean dPadDown  = gamepad1.dpad_down;
        boolean dPadLeft  = gamepad1.dpad_left;
        boolean dPadRight = gamepad1.dpad_right;
        //bumperPressed = gamepad1.right_bumper;
        //telemetry.addData("Gyro", angleDouble);
        //telemetry.update();
        if(countUp){
            if(countsinceapressed < 10){
                countsinceapressed++;
            }
            else{
                countUp = false;
                countsinceapressed = 0;
            }
        }
        if(buttonAPressed&& !countUp) {
            countUp = true;
            if (speedMode == false) {
                speedMode = true;
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Mode", "Speed");
                telemetry.update();
            } else if (speedMode == true) {
                speedMode = false;
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Mode", "Power");
                telemetry.update();
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
        if(gamepad1.y) {
            lezGoSlow = true;
        }
        else {
            lezGoSlow = false;
        }

        if(buttonXPressed == true){
            offset = Double.parseDouble(angleDouble);
            offset = -offset;
        }
        calculator.calculateMovement(leftX, leftY, rightX);
        if(lezGoSlow) {
            if (!speedMode) {
                leftMotor.setPower(.1 * calculator.getLeftDrive());
                rightMotor.setPower(.1 * calculator.getRightDrive());
                middleMotor.setPower(.2*-calculator.getMiddleDrive());
            } else {
                leftMotor.setPower(.1*calculator.getLeftDrive());
                rightMotor.setPower(.1*calculator.getRightDrive());
                middleMotor.setPower(.2*-calculator.getMiddleDrive());
            }
        }
        else {
            double scale = .6;
            if (!speedMode) {
                leftMotor.setPower(scale * calculator.getLeftDrive());
                rightMotor.setPower(scale * calculator.getRightDrive());
                middleMotor.setPower(.8*-calculator.getMiddleDrive());
            } else {
                leftMotor.setPower(scale*calculator.getLeftDrive());
                rightMotor.setPower(scale*calculator.getRightDrive());
                middleMotor.setPower(.8-calculator.getMiddleDrive());
            }
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
        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.update();
        if(gamepad1.x) {
            arm.setPower(-.3); //4900 encoder arm counts
        }
        else if(gamepad1.y) {
            arm.setPower(.3);
        }
        else {
            arm.setPower(0);
        }
        if(gamepad1.left_trigger == 1) {
            hirshIsDumb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{}
        if(gamepad1.left_bumper == true) {
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            claw1.setPosition(.8);
            claw2.setPosition(.35);
            stateGlyph = true;
            hirshIsDumb.setPower(.35);
            hirshIsDumb.setTargetPosition(700);
        }
        else if(gamepad1.right_bumper == true) {
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            claw1.setPosition(.55);
            claw2.setPosition(.55);
            stateGlyph = false;
            hirshIsDumb.setPower(-.20);
            hirshIsDumb.setTargetPosition(100);
        }
        else if(gamepad1.dpad_down) {
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hirshIsDumb.setPower(-.20);
        }
        else if(gamepad1.dpad_up) {
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hirshIsDumb.setPower(.35);
        }
        else {
            hirshIsDumb.setPower(0);
        }
        if(stateGlyph == true && gamepad1.right_trigger == 1) {
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(glyphCounter >= 10) {
                stateGlyph = false;
                claw1.setPosition(.2);
                claw2.setPosition(.8);
                glyphCounter = 0;
            }
        }
        if(stateGlyph == false && gamepad1.right_trigger == 1) {
            hirshIsDumb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(glyphCounter >= 10) {
                claw1.setPosition(.75);
                claw2.setPosition(.4);
                stateGlyph = true;
                glyphCounter = 0;
            }
        }
        //****************************************************************
        /*IntakeServo= hardwareMap.servo.get("IntakeServo");
        if (gamepad2.right_bumper == true){
            //IntakeServo.setPosition(1);
        }
        else {//IntakeServo.setPosition(0);
        }
       IntakeMotor= hardwareMap.dcMotor.get("IntakeMotor");
        if (gamepad2.left_bumper == true){
            IntakeMotor.setPower(1);
        }
        else {IntakeMotor.setPower(0);
        }
*/
        //.addData("Gyro",Double.parseDouble(angleDouble) + offset);
        //telemetry.update();
        //***********************************************************************
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));


    }
}

