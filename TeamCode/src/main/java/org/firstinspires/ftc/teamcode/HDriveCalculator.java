package org.firstinspires.ftc.teamcode;

/**
 * Created by ian on 8/18/2016.
 */

//test comment Luke ... ...
//Github test comment
    //hirsh ramani is NOT cool
    //Github test comment 2
    //Github test comment 3
//test comment 4
public class HDriveCalculator {
    double driveLeft;
    double driveRight;
    double driveMiddle;
    public HDriveCalculator(){

    }
    public void calculateMovement(double joystickX, double joystickY, double turnStickX, double angle){
        if(joystickX != 0 && joystickY != 0) {
            double scaleFactor = Math.sqrt(joystickX * joystickX + joystickY * joystickY);
            double moveAngle;
            if(joystickX > 0){
                if(joystickY > 0) {
                    moveAngle = Math.atan(joystickY / joystickX);
                }
                else{
                    moveAngle = 3/2 * Math.PI - Math.atan(joystickY / joystickX);
                }
            }
            else{
                        if(joystickY > 0){
                    moveAngle = 1/2 * Math.PI + (1/2 * Math.PI + Math.atan(joystickY / joystickX));
                }
                        else{
                    moveAngle = Math.PI + Math.atan(joystickY / joystickX);
                }
            }
            double robotAngle = angle;
            double xMove = scaleFactor * (Math.cos(moveAngle - robotAngle));
            double yMove = scaleFactor * Math.sin(moveAngle - robotAngle);
            double turn = turnStickX / 2;
            driveLeft = yMove + turn;
            driveRight = yMove - turn;
            driveMiddle = xMove;
            if (driveLeft > 1) {
                driveRight = driveRight * (1 / driveLeft);
                driveMiddle = driveMiddle * (1 / driveLeft);
                driveLeft = 1;
            } else if (driveRight > 1) {
                driveLeft = driveLeft * (1 / driveRight);
                driveMiddle = driveMiddle * (1 / driveRight);
                driveRight = 1;
            }
        }
        else if(joystickX == 0 && joystickY != 0){
            double scaleFactor = Math.abs(joystickY);
            double moveAngle;
            if(joystickY > 0) {
                moveAngle = Math.PI/2;
            }
            else{
                moveAngle = 3/2 * Math.PI;
            }
            double robotAngle = angle;
            double xMove = scaleFactor * (Math.cos(moveAngle - robotAngle));
            double yMove = scaleFactor * Math.sin(moveAngle - robotAngle);
            double turn = turnStickX / 2;
            driveLeft = yMove + turn;
            driveRight = yMove - turn;
            driveMiddle = xMove;
            if (driveLeft > 1) {
                driveRight = driveRight * (1 / driveLeft);
                driveMiddle = driveMiddle * (1 / driveLeft);
                driveLeft = 1;
            } else if (driveRight > 1) {
                driveLeft = driveLeft * (1 / driveRight);
                driveMiddle = driveMiddle * (1 / driveRight);
                driveRight = 1;
            }
        }
        else if(joystickY == 0 && joystickX != 0){
            double scaleFactor = Math.abs(joystickX);
            double moveAngle;
            if(joystickX > 0) {
                moveAngle = 0;
            }
            else{
                moveAngle = Math.PI;
            }
            double robotAngle = angle;
            double xMove = scaleFactor * (Math.cos(moveAngle - robotAngle));
            double yMove = scaleFactor * Math.sin(moveAngle - robotAngle);
            double turn = turnStickX / 2;
            driveLeft = yMove + turn;
            driveRight = yMove - turn;
            driveMiddle = xMove;
            if (driveLeft > 1) {
                driveRight = driveRight * (1 / driveLeft);
                driveMiddle = driveMiddle * (1 / driveLeft);
                driveLeft = 1;
            } else if (driveRight > 1) {
                driveLeft = driveLeft * (1 / driveRight);
                driveMiddle = driveMiddle * (1 / driveRight);
                driveRight = 1;
            }
        }
        else{
            double turn = turnStickX / 2;
            driveLeft = turn;
            driveRight = -turn;
            driveMiddle = 0;
        }
    }
    public double getLeftDrive(){
        return driveLeft;
    }
    public double getRightDrive(){
        return driveRight;
    }
    public double getMiddleDrive(){
        return driveMiddle;
    }
}
