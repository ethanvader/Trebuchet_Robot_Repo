package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@Autonomous()
public class TrebuchetAuton extends LinearOpMode {
     private Johnny6 johnny6;

     private ElapsedTime runtime=new ElapsedTime();

    @Override
    public void runOpMode() {
        johnny6 = new Johnny6(this, Johnny6.Drivetrain.TREBUCHET6);
        runtime.reset();

        waitForStart();

        double speed = 0.5;
        int rest = 100;


    }
}
