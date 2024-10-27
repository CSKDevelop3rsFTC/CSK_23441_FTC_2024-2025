package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "autoBasic")
public class autoBasic extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    //4 Stage Slides Motors (Outtake)
    public DcMotor outtakeMotor1 = null;
    public DcMotor outtakeMotor2 = null;

    //Intake Servos
    public Servo hzfourbarServo1 = null;
    public Servo hzfourbarServo2 = null;
    public Servo hzSlidesServo1 = null;
    public Servo hzSlidesServo2 = null;
    public CRServo intakeServo = null;

    //Outtake Servos
    public Servo vFourbarServo1 = null;
    public Servo vFourbarServo2 = null;
    public Servo clawRotateServo = null;
    public Servo clawServo = null;

    public double driveSpeed = 0.5;


    public void stopRobot () {
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    public void vertMove ( double dir, int time) { //inputs for dir is -1 or 1 (forward or backward)
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setPower(driveSpeed * dir);
        frontLeftDrive.setPower(driveSpeed * dir);
        backRightDrive.setPower(driveSpeed * dir);
        backLeftDrive.setPower(driveSpeed * dir);
        sleep(time);
        }

    public void horizMove ( double dir, int time){ // -1 for left 1 for positive  for
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

    public void pivot ( double dir, int time){ // -1 for turnLeft 1 for turnRight
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

    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        outtakeMotor1 = hardwareMap.get(DcMotor.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "outtakeMotor2");

        hzfourbarServo1 = hardwareMap.get(Servo.class, "hzfourbarServo1");
        hzfourbarServo2 = hardwareMap.get(Servo.class, "hzfourbarServo2");
        hzSlidesServo1 = hardwareMap.get(Servo.class, "hzSlidesServo1");
        hzSlidesServo2 = hardwareMap.get(Servo.class, "hzSlidesServo2");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        vFourbarServo1 = hardwareMap.get(Servo.class, "vFourbarServo1");
        vFourbarServo2 = hardwareMap.get(Servo.class, "vFourbarServo2");
        clawRotateServo = hardwareMap.get(Servo.class, "clawRotateServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        telemetry.addData("Status", "Ready to Start");
        telemetry.update();
        waitForStart();

        vertMove(1, 800);

        stopRobot();

    }



}
