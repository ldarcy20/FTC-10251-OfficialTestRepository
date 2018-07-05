package org.firstinspires.ftc.teamcode;

/**
 * Created by Luke on 9/7/2016.
 */
public class HDriveNonFeildCentric {
    double leftMove;
    double rightMove;
    double middleMove;
    double angle = 0;
    public void calculateMovement (double joystickX, double joystickY, double turnStickX) {
        double robotAngle = angle/360 * 2 * Math.PI;
        double tempVal = joystickY * Math.cos(robotAngle) - joystickX * Math.sin(robotAngle);
        double driveMiddle = joystickX * Math.cos(robotAngle) - joystickY * Math.sin(robotAngle);
        double driveLeft = tempVal + turnStickX/2;
        double driveRight = tempVal - turnStickX/2;
        if (driveLeft > 1) {
            driveRight = driveRight * (1 / driveLeft);
            driveMiddle = driveMiddle * (1 / driveLeft);
            driveLeft = 1;
        } else if (driveRight > 1) {
            driveLeft = driveLeft * (1 / driveRight);
            driveMiddle = driveMiddle * (1 / driveRight);
            driveRight = 1;
        }
        leftMove = driveLeft;
        rightMove = driveRight;
        middleMove = driveMiddle;
    }
    public double getLeftDrive(){
        return leftMove;
    }
    public double getRightDrive(){
        return rightMove;
    }
    public double getMiddleDrive(){
        return middleMove;
    }
}
