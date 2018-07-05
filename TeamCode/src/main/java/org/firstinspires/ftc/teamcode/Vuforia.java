package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;/**
 * Created by Luke on 11/9/2016.
 */
@Disabled
@Autonomous(name= "Vuforia", group = "Vision Assist")
public class Vuforia extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AdauJ9r/////AAAAGQ8T4+ND3kBNj3ASq3Yz50YOL+BYwmGKT4RocYSdmEwHIwh0JAjJ2xBBwv4n3+XZObZ700pc3hmcmq+ySQEVAnKkPGq7aCpOtkGTlxQ+EJlbEu3BhSdjSeBPqYC3bxd8cEDUkdCAWCzshMdeXgknPQYwZZgzGCQAFrZquqDkiIvcq9zefTCXEO/NYjS7uC1yUQZD5wJGBBStkCQpVzg9TqzWVn6qh27dDEpfhU2VrSHh29cjMO7UWAneARLViJ7EV8337KNU5GIcn5y4AUNNXbc2M73nUOvGLeMxESvPciGdnejTTe4AG/G53LrFn1UNJB0qTg4dkkbxmjQjixbjY/P57O4TC+woidwZJpsMKkEF";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while(opModeIsActive()) {
            for(VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if(pose != null) {
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(1 + "-Translation", translation.get(2)); //

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(2),translation.get(0))); //If phone is vertical, if landscape, then change first translation to translation(0)
                    telemetry.addData(1 + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();

        }
    }
}
