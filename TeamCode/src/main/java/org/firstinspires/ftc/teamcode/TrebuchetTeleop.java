package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="OutreachTeleop")
public class TrebuchetTeleop extends OpMode {
    Johnny6 trebuchet6;
    boolean unLocked=false;
    boolean rTriggerPressed=false;
    boolean rTriggerProcessed=false;
    boolean sensorPressed=false;
    boolean sensorNotPressed=false;
    boolean sensorProcessed=false;

    @Override
    public void init() { trebuchet6=new Johnny6( this, Johnny6.Drivetrain.TREBUCHET6);
    trebuchet6.unLock();}

    public void loop(){

        //Movement code
        double y=-gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x;

        /*y*=y;
        x*=x;
        if(gamepad1.left_stick_y>0){
            y=-y;
        }
        if(gamepad1.left_stick_x<0){
            x=-x;
        }

        double turn=gamepad1.left_stick_x/2;
        trebuchet6.move(-x,y,0); */

        //Moves the trebuchet forward
        if(gamepad1.left_stick_y>0){
            trebuchet6.motorFrontRight.setPower(1);
            trebuchet6.motorFrontLeft.setPower(1);
        }
        else {
            trebuchet6.motorFrontRight.setPower(0);
            trebuchet6.motorFrontLeft.setPower(0);
        }
        //Moves the trebuchet backward
        if(gamepad1.left_stick_y<0){
            trebuchet6.motorFrontRight.setPower(-1);
            trebuchet6.motorFrontLeft.setPower(-1);
        }
        else {
            trebuchet6.motorFrontRight.setPower(0);
            trebuchet6.motorFrontLeft.setPower(0);
        }
        //Turns the trebuchet to the right
        if(gamepad1.right_stick_x>0){
            trebuchet6.motorFrontRight.setPower(-1);
            trebuchet6.motorFrontLeft.setPower(1);
        }
        else {
            trebuchet6.motorFrontRight.setPower(0);
            trebuchet6.motorFrontLeft.setPower(0);
        }
        //Turns the trebuchet to the left
        if(gamepad1.right_stick_x<0){
            trebuchet6.motorFrontRight.setPower(1);
            trebuchet6.motorFrontLeft.setPower(-1);
        }
        else {
            trebuchet6.motorFrontRight.setPower(0);
            trebuchet6.motorFrontLeft.setPower(0);
        }


        //Code for trebuchet launch
        if(gamepad1.right_trigger > 0.2){
            rTriggerPressed=true;
        }else{
            rTriggerPressed=false;
            rTriggerProcessed=false;
        }
        if(rTriggerPressed&&!rTriggerProcessed){
            if(unLocked){
                trebuchet6.Locked();
                unLocked=false;
            }
            else{
                trebuchet6.unLock();
                unLocked=true;
                sensorPressed=false;
            }
            rTriggerProcessed=true;
        }

        //codings for the stupid sensor we cant figure out
        if(trebuchet6.isLockSensorPressed() && !sensorPressed) {
            sensorPressed=true;
            if (sensorPressed){
                trebuchet6.Locked();
            }

        }

    }
}

