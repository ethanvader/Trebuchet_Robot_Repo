package org.firstinspires.ftc.teamcode;

import static java.nio.file.Files.move;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorTouch;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class Johnny6 {

    private HardwareMap hwMap;

    LinearOpMode auton;

    OpMode teleop;
    public enum Drivetrain {
        MECHANUM,
        JOHNNY6,
        TEST,
        TREBUCHET6
    }

    public enum Team {
        RED,
        BLUE
    }

    private Drivetrain drive;

    private Telemetry telem;

    //Definitions for global variables

    public DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    //[] means array
    public DcMotor[] allDriveMotors;

    //Outreach robot servos
    public Servo lockServo;
    // for outreach
    public TouchSensor lockSensor;

    //public CRServo //future necessary robot functions using servos
    private IMU imu;

    private IMU.Parameters parameters;

    public WebcamName webcamName;

    //Put any CONSTANTS here

    static final double Y_INCH_TICKS = 45;

    static final double X_INCH_TICKS = 45;

    static final double X_DEGREE_TICKS = 11.1; //may need to be changed

    static final double Y_DEGREE_TICKS = 11.1; //may need to be changed

    public Johnny6(OpMode opmode, Drivetrain drivetrain) {

        this.teleop = opmode;

        this.hwMap = opmode.hardwareMap;

        this.drive = drivetrain;

        this.telem = opmode.telemetry;

        //setUpHardware maps variables to their hardware object
        setupHardware();
    }

    public Johnny6(LinearOpMode opmode, Drivetrain type) {

        this.auton = opmode;

        hwMap = opmode.hardwareMap;

        telem = opmode.telemetry;

        drive = type;

        setupHardware();

        telem.addLine("init motor test");
    }

    public Johnny6(HardwareMap hardwareMap, Drivetrain drivetrain) {

        this.hwMap = hardwareMap;

        this.drive = drivetrain;

        setupHardware();
    }

    private void setupHardware() {

        //This switch statement is used to choose which drivetrain
        //depending on the drive variable
        switch (drive) {

            case TEST:

                //setup motors for drivetrain
                motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
                motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
                motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
                motorBackRight = hwMap.dcMotor.get("motorBackRight");

                //Reverse motors
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

                //Here would go any additional hardware devices for the robot

                imu = hwMap.get(IMU.class, "imu");

                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

                imu.initialize(parameters);


                //camera setup!
                webcamName = hwMap.get(WebcamName.class, "eyeofjohnny6");
                allDriveMotors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};

                break;

            case TREBUCHET6:

                motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
                motorFrontRight = hwMap.dcMotor.get("motorFrontRight");

                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


                imu = hwMap.get(IMU.class, "imu");

                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

                imu.initialize(parameters);

                //these are for Trebuchet
                lockServo = hwMap.servo.get("lockServo");
                lockSensor = hwMap.touchSensor.get("lockSensor");

                break;

            default:

                telem.addLine("Invalid type " + drive + " passed to Johnny6's init function. Nothing has been set up ");
                break;
        }
    }

    //Set motor power for all drivetrain motors on robot to 0

    //set powers for motors and positions for servos
    public void rest() {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
    }

    /*
    This function controls movement for the robot.
    @param x the x speed value
    @param y the y speed value
    @param turn the turn speed value
     */

    public void move(double x, double y, double turn) {

        switch (drive) {

            case MECHANUM:

                //Denominator is the larget motor power (absolute value) or 1
                //This ensures all the powers maintain the same ratio, but only when
                //at least one is out of the range [-1, 1]
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

                //Compute values for the power of each motor
                frontLeftPower = (y + x + turn) / denominator;
                backLeftPower = (y - x + turn) / denominator;
                frontRightPower = (y - x - turn) / denominator;
                backRightPower = (y + x - turn) / denominator;

                //Assign that motor power to each motor
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                break;

            case TREBUCHET6:
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

                //compute the values for power of each motor
                // make -y because if it was on front, it would be backwards

                backLeftPower = (-y - x) / denominator;
                backRightPower = (-y + x) / denominator;

                //assign power to motor
                motorFrontLeft.setPower(backLeftPower);
                motorFrontRight.setPower(backRightPower);

                break;
        }
    }


    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    //Trebuchet servos and other thingies
    public void unLock() {
        lockServo.setPosition(0.5);
    }

    public void Locked() {
        lockServo.setPosition(1);
    }

    public boolean isLockSensorPressed() {
        return lockSensor.isPressed();
    }

    public boolean isBottomSensorPressed() {
        return bottomSensor.isPressed();
    }

    public void rotateTo(int tickTarget){
            clawMotor.setTargetPosition(tickTarget);
            clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawMotor.setPower(1);
    };

        //Converts to integer by rounding. CASTS to int after rounding.
        int tickTarget = (int) Math.round(-inches * Y_INCH_TICKS);

        resetDriveEncoders();

        for (DcMotor x : allDriveMotors) {

            x.setTargetPosition(tickTarget);
            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        move(0, -speed, 0);

        waitForMotors();

        resetDriveEncoders();

    }

    public void moveBackwardInches(double inches, double speed) {

        moveForwardInches(-inches, -speed);

    }

    public void moveRightInches(double inches, double speed) {

        int tickTarget = (int) Math.round(inches * X_INCH_TICKS);

        resetDriveEncoders();

        motorFrontLeft.setTargetPosition(tickTarget);
        motorFrontRight.setTargetPosition(-tickTarget);
        motorBackLeft.setTargetPosition(-tickTarget);
        motorBackRight.setTargetPosition(tickTarget);


        for (DcMotor x : allDriveMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        move(speed, 0, 0);

        waitForMotors();

        resetDriveEncoders();

    }

    public void moveLeftInches(double inches, double speed) {

        moveRightInches(-inches, -speed);
    }

    public void turnRightDegrees(double degrees, double speed) {
        int tickTarget = (int) Math.round(degrees * X_DEGREE_TICKS);

        resetDriveEncoders();

        motorFrontLeft.setTargetPosition(tickTarget);
        motorFrontRight.setTargetPosition(-tickTarget);
        motorBackLeft.setTargetPosition(tickTarget);
        motorBackRight.setTargetPosition(-tickTarget);

        for (DcMotor x : allDriveMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        move(0, 0, speed);

        waitForMotors();

        resetDriveEncoders();

    }

    public void turnLeftDegrees(double degrees, double speed) {
        turnRightDegrees(-degrees, -speed);
    }


    public void waitForMotors() {
        boolean finished = false;
        while (auton.opModeIsActive() && !finished && !auton.isStopRequested()) {
                if (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                    telem.addData("front left encoder:", motorFrontLeft.getCurrentPosition());
                    telem.addData("front right encoder:", motorFrontRight.getCurrentPosition());
                    telem.addData("back left encoder:", motorBackLeft.getCurrentPosition());
                    telem.addData("back right encoder:", motorBackRight.getCurrentPosition());

                    telem.update();
                } else {
                    finished = true;
                }
        }
    }

    private void resetDriveEncoders() {
        for (DcMotor x: allDriveMotors) {
            x.setPower(0);
            x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            x.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }