package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "autoPrototype")
public class autoPrototype extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    //4 Stage Slides Motors (Outtake)
    public DcMotor  outtakeMotor1    = null;
    public DcMotor  outtakeMotor2    = null;

    //Intake Servos
    public Servo    hzfourbarServo1  = null;
    public Servo    hzfourbarServo2  = null;
    public Servo    hzSlidesServo1   = null;
    public Servo    hzSlidesServo2   = null;
    public Servo    intakeServo      = null;

    //Outtake Servos
    public Servo    vFourbarServo1   = null;
    public Servo    vFourbarServo2   = null;
    public Servo    clawRotateServo  = null;
    public Servo    clawServo        = null;

    public double driveSpeed = 0.5;
 /*   public void initialize(){


    }*/

    public void stopRobot() {
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    public void vertMove(double dir, int time){ //inputs for dir is -1 or 1 (forward or backward)
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setPower(driveSpeed * dir);
        frontLeftDrive.setPower(driveSpeed * dir);
        backRightDrive.setPower(driveSpeed * dir);
        backLeftDrive.setPower(driveSpeed * dir);
        sleep(time);
    }
    public void horizMove(double dir, int time){ // -1 for left 1 for positive  for
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setPower(driveSpeed * dir);
        frontLeftDrive.setPower(driveSpeed * dir);
        backRightDrive.setPower(driveSpeed * dir);
        backLeftDrive.setPower(driveSpeed * dir);
        sleep(time);
    }
    public void pivot(double dir, int time){ // -1 for turnLeft 1 for turnRight
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setPower(driveSpeed * dir);
        frontLeftDrive.setPower(driveSpeed * dir);
        backRightDrive.setPower(driveSpeed * dir);
        backLeftDrive.setPower(driveSpeed * dir);
        sleep(time);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        vertMove(1, 500);
        horizMove(1,500);
        vertMove(-1,300);
        vertMove(-1,300);
        pivot(1,600);
        pivot(-1,300);

        stopRobot();

    }
}
