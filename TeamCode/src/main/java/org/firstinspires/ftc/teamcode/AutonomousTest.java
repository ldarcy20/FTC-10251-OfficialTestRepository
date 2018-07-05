package org.firstinspires.ftc.teamcode;
import android.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by Luke on 9/25/2016.
 */
@Autonomous(name= "Autonomous Cypher", group = "HDrive")
public class AutonomousTest extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    Orientation angles;
    Orientation angles2;
    BNO055IMU imu;
    OpticalDistanceSensor opticalDistanceSensor;
    private ElapsedTime runtime = new ElapsedTime();// Use a Pushbot's hardware
    static final double     COUNTS_PER_MOTOR_REV    = 1040 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4; ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    TouchSensor             touchsensor;
    ColorSensor             sensorRGB;
    AnalogInput distanceSensor;// Use a HDrive's hardware
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor middleMotor;
    DcMotor glyphMotor;
    Servo claw1;
    Servo claw2;
    Servo arm;
    Servo servo;
    double gyroDouble = 0;
    double initialAngle = 0;
    Servo servo2;
    int[] yowassup = new int[13];
    @Override public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdauJ9r/////AAAAGQ8T4+ND3kBNj3ASq3Yz50YOL+BYwmGKT4RocYSdmEwHIwh0JAjJ2xBBwv4n3+XZObZ700pc3hmcmq+ySQEVAnKkPGq7aCpOtkGTlxQ+EJlbEu3BhSdjSeBPqYC3bxd8cEDUkdCAWCzshMdeXgknPQYwZZgzGCQAFrZquqDkiIvcq9zefTCXEO/NYjS7uC1yUQZD5wJGBBStkCQpVzg9TqzWVn6qh27dDEpfhU2VrSHh29cjMO7UWAneARLViJ7EV8337KNU5GIcn5y4AUNNXbc2M73nUOvGLeMxESvPciGdnejTTe4AG/G53LrFn1UNJB0qTg4dkkbxmjQjixbjY/P57O4TC+woidwZJpsMKkEF";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        telemetry.addData(">", "created by the best programmer adam");
        telemetry.addData(">", "Press play to start");
        telemetry.update();

        float hsvValues[] = {0F,0F,0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        int timeStart = 0;
        int timeEnd = 0;
        int encodersRunning = 0;
        int gyroIsSuperDumbHole = 0;

        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        //touchsensor = hardwareMap.touchSensor.get("touch_sensor");
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        BNO055IMU.Parameters parameters3 = new BNO055IMU.Parameters();
        parameters3.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters3.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters3.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters3.loggingEnabled      = true;
        parameters3.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters3);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        leftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftMotor");
        rightMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightMotor");
        middleMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "middleMotor");
        glyphMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "Hirsh is very dumb");
        claw1 = hardwareMap.servo.get("claw1");
        claw2 = hardwareMap.servo.get("claw2");
        //controller = hardwareMap.dcMotorController.get("Motor Controller 1");
        //controller2 = hardwareMap.dcMotorController.get("Motor Controller 2");
        //controller3 = hardwareMap.servoController.get("Servo Controller 1");
        //distanceSensor = hardwareMap.analogInput.get("ODS");
        //servo2 = hardwareMap.servo.get("servo2");
        //sensorRGB = hardwareMap.colorSensor.get("color");

            /*
         * Initialize the drive system variables.
         * The init
         * () method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders for awesome reason");    //
        telemetry.update();
        //leftMotor.setDirection(DcMotor.Direction.REVERSE);distanceSensor = hardwareMap.analogInput.get("ODS");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition(),
                middleMotor.getCurrentPosition());
        telemetry.update();
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        middleMotor.setDirection(DcMotor.Direction.FORWARD);
        distanceSensor = hardwareMap.analogInput.get("ODS");

        waitForStart();
        /*servo3.setPosition(1);
        Thread.sleep(4000);
        arm.setPosition(1);
        Thread.sleep(2000);
        servo3.setPosition(1);
        Thread.sleep(2000);
        servo3.setPosition(0);
        Thread.sleep(2000);
        arm.setPosition(0);
        Thread.sleep(2000);
        servo3.setPosition(0);
        servo3.setPosition(0);
        Thread.sleep(60000);*/
        relicTrackables.activate();
        int i = 0;
        leftMotor.setPower(.1);
        rightMotor.setPower(.1);
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        while(vuMark == RelicRecoveryVuMark.UNKNOWN) {
            i++;
            leftMotor.setPower(.1);
            rightMotor.setPower(.1);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuMark", "%s visible", vuMark);
                telemetry.addData("right Motor", rightMotor.getCurrentPosition());
                telemetry.addData("left Motor", leftMotor.getCurrentPosition());
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                break;
            } else {
                telemetry.addData("VuMark", "not visible!");
                telemetry.addData("Loops", i);
            }

            telemetry.update();
        }
        angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        while(angles2.firstAngle > 0) {
            angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            telemetry.addLine("You did it congrats");
            telemetry.update();
        }
        telemetry.addLine("Done Boi");
        telemetry.update();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.addData("VuMark",vuMark);
        telemetry.update();
        int cycles = 0;
        angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftMotor.setPower(.12);
        rightMotor.setPower(.12);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", angles.firstAngle);
        telemetry.update();
        leftMotor.setPower(-.2);
        rightMotor.setPower(.2);

        encoderDrive(-.5,3,3,60);
        Thread.sleep(150);
        encoderDriveMiddle(.5,12,60);
        Thread.sleep(150);
        leftMotor.setPower(.6);
        rightMotor.setPower(.6);
        while(distanceSensor.getVoltage() < .4) {
            telemetry.addData("Distance", distanceSensor.getVoltage());
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        Thread.sleep(200);
        encoderDriveMiddle(-.7,-10,60);
        i = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Gyro Angle", angles.firstAngle);
        telemetry.update();
        leftMotor.setPower(.4);
        rightMotor.setPower(-.4);
        while(angles.firstAngle > -80) {
            angles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
            double coeffecient = (90-Math.abs(angles.firstAngle))/90;
            if(coeffecient > .1) {
                leftMotor.setPower(.5 * (coeffecient * 1));
                rightMotor.setPower(-(coeffecient * .5));
            }
            else {
                leftMotor.setPower(1.3);
                rightMotor.setPower(-1.3);
            }
            telemetry.addData("Gyro Angle", angles.firstAngle);
            telemetry.addData("Looped", coeffecient);
            telemetry.update();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles2.firstAngle != angles.firstAngle) {
                gyroIsSuperDumbHole++;
            }
            i++;
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        vuMark = RelicRecoveryVuMark.LEFT;
        telemetry.addData("Kill Hirsh PLs", vuMark);
        telemetry.update();
        if(vuMark == RelicRecoveryVuMark.LEFT) {
            encoderDriveMiddle(-.5,-12,60);
        }
        else if(vuMark == RelicRecoveryVuMark.RIGHT) {
            encoderDriveMiddle(.5,12,60);
        }
        else {
        }

    }
    public void adjust(float angleStart) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(angles.firstAngle < angleStart) {
            leftMotor.setPower(.01);
            rightMotor.setPower(-.01);
        }
        else if(angles.firstAngle > angleStart) {
            leftMotor.setPower(-.01);
            rightMotor.setPower(.01);
        }
        else if(angles.firstAngle == angleStart) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        else {
            telemetry.addLine("WTF");
            telemetry.update();
        }
    }
    public void encoderDriveMiddle(double speed,//int leftDistance, int rightDistance,
                                   double middleInches,
                                   double timeoutS) throws InterruptedException {

        int newMiddleTarget = 0;
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d", newMiddleTarget);
                //telemetry.addData("Path2", "Running at %7d",
                //        middleMotor.getCurrentPosition());
                //telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrive(double speed,//int leftDistance, int rightDistance,
                                  double leftInches, double rightInches,
                                  double timeoutS) throws InterruptedException {



        int newLeftTarget = 0;
        int newRightTarget = 0;
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Encoder Left", leftMotor.getCurrentPosition());
                telemetry.addData("Encoder Right", rightMotor.getCurrentPosition());
                //leftMotor.getCurrentPosition(),
                //rightMotor.getCurrentPosition();

                // telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveMotionProfileing(double speed,//int leftDistance, int rightDistance,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {



        int newLeftTarget = 0;
        int newRightTarget = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int)((leftInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.rightMotor.getCurrentPosition() + */(int)((rightInches) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);
            int speedWeWant = 0;
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy() )) {
                if(leftMotor.getCurrentPosition() == (newLeftTarget/3)*2) {

                }
                // Display it for the driver.
                telemetry.addData("Encoder Left", leftMotor.getCurrentPosition());
                telemetry.addData("Encoder Right", rightMotor.getCurrentPosition());
                //leftMotor.getCurrentPosition(),
                //rightMotor.getCurrentPosition();

                // telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    public void encoderDriveBoth(double speed, double speedMiddle,//int leftDistance, int rightDistance,
                                 double middleInches, double leftInches, double rightInches,
                                 double timeoutS) throws InterruptedException {

        int newMiddleTarget = 0;
        int newLeftTarget = 0;
        int newRightTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMiddleTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((middleInches) * COUNTS_PER_INCH);
            newLeftTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((rightInches) * COUNTS_PER_INCH);
            newRightTarget = /*robot.leftMotor.getCurrentPosition() + */(int) ((leftInches) * COUNTS_PER_INCH);
            middleMotor.setTargetPosition(newMiddleTarget);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            // Turn On RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            middleMotor.setPower(speedMiddle);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (middleMotor.isBusy() && leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d", newMiddleTarget);
                //telemetry.addData("Path2", "Running at %7d",
                //        middleMotor.getCurrentPosition());
                //telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            middleMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            //  sleep(250);   // optional pause after each move
        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
}

}
