package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Luke on 9/25/2016.
 */
public class HardwareHDrive {

        /* Public OpMode members. */
        public DcMotor leftMotor   = null;
        public DcMotor  rightMotor  = null;
        public DcMotor middleMotor = null;



        /* local OpMode members. */
        HardwareMap hwMap           =  null;
        private ElapsedTime period  = new ElapsedTime();

        /* Constructor */
        public HardwareHDrive(){

        }

        /* Initialize standard Hardware interfaces */
        public void init(HardwareMap ahwMap) {
            // Save reference to Hardware map
            hwMap = ahwMap;

            // Define and Initialize Motors
            leftMotor   = hwMap.dcMotor.get("leftMotor");
            rightMotor  = hwMap.dcMotor.get("rightMotor");
            middleMotor = hwMap.dcMotor.get("middleMotor");
            leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            middleMotor.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);


            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
        ;

            // Define and initialize ALL installed servos.

        }

        /***
         *
         * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
         * periodic tick.  This is used to compensate for varying processing times for each cycle.
         * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
         *
         * @param periodMs  Length of wait cycle in mSec.
         * @throws InterruptedException
         */




}
