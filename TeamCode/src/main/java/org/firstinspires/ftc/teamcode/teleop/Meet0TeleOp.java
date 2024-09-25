package org.firstinspires.ftc.teamcode.teleop;

//import statements

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MeetZero:Teleop", group="Robot")
public class Meet0TeleOp extends LinearOpMode {

    //Drive Train Motors
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;

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

    public void initialize(){
        frontLeftDrive    =    hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive   =   hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive     =     hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive    =    hardwareMap.get(DcMotor.class, "backRightDrive");

        outtakeMotor1     =     hardwareMap.get(DcMotor.class, "outtakeMotor1");
        outtakeMotor2     =     hardwareMap.get(DcMotor.class, "outtakeMotor2");

        hzfourbarServo1   =     hardwareMap.get(Servo.class, "hzfourbarServo1");
        hzfourbarServo2   =     hardwareMap.get(Servo.class, "hzfourbarServo2");
        hzSlidesServo1    =      hardwareMap.get(Servo.class, "hzSlidesServo1");
        hzSlidesServo2    =      hardwareMap.get(Servo.class, "hzSlidesServo2");
        intakeServo       =         hardwareMap.get(Servo.class, "intakeServo");

        vFourbarServo1    =      hardwareMap.get(Servo.class, "vFourbarServo1");
        vFourbarServo2    =      hardwareMap.get(Servo.class, "vFourbarServo2");
        clawRotateServo   =     hardwareMap.get(Servo.class, "clawRotateServo");
        clawServo         =           hardwareMap.get(Servo.class, "clawServo");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //outtakeServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void driveMovement(double driveSpeed){
        double vertical;
        double horizontal;
        double pivot;

        horizontal =    gamepad1.left_stick_x;
        vertical   =   -gamepad1.left_stick_y;
        pivot      =   gamepad1.right_stick_x;

        frontRightDrive.setPower(((vertical + horizontal)-pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal)+pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal)-pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal)+pivot) * driveSpeed);
    }

    public void driveController() {

        if (gamepad1.left_bumper) {
            driveMovement(1);
        }

        else {
            driveMovement(0.75);
        }

    }

    public void run(double verticalPosition, double horizontalPosition, double pivot){

        double y = -verticalPosition; // Remember, Y stick value is reversed
        double x = horizontalPosition; // Counteract imperfect strafing
        double rx = pivot;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower  = (-y - x -rx)  / denominator;
        double backLeftPower   = (-y + x - rx) / denominator;
        double frontRightPower = (y - x - rx)  / denominator;
        double backRightPower  = (y + x - rx)  / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            driveController();

            sleep(50);

        }

    }

}
