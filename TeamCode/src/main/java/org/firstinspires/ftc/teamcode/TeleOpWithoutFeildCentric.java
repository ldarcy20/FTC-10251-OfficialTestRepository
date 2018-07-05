package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp: Non-Feild Centric", group="HDrive")
@Disabled
public class TeleOpWithoutFeildCentric extends LinearOpMode {

    /* Declare OpMode members. */
    TeleOpNotFieldCentric robot           = new TeleOpNotFieldCentric();              // Use a K9'shardware


    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double middle;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.left_stick_y;
            middle = -gamepad1.right_stick_x;

            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            robot.middleMotor.setPower(middle);

            // Use gamepad Y & A raise and lower the arm


            // Use gamepad X & B to open and close the claw

            // Move both servos to new position.


            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("middle", "%.2f", middle);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
