package org.firstinspires.ftc.teamcode.teleop;

//import

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

    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;
    public Servo    outtakeServo     = null;

    public void initialize(){
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        outtakeServo  = hardwareMap.get(Servo.class, "outtakeServo");
        outtakeServo.setDirection(Servo.Direction.FORWARD);
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();
    }

    public void driveTrainMovement(){
        double vertical;
        double horizontal;
        double pivot;
        double max;

        horizontal =  gamepad1.left_stick_x;
        vertical   = gamepad1.left_stick_y;
        pivot      = -gamepad1.right_stick_x;

        frontRightDrive.setPower(((vertical + horizontal)-pivot) * 0.75);
        frontLeftDrive.setPower(((vertical - horizontal)+pivot) * 0.75);
        backRightDrive.setPower(((vertical - horizontal)-pivot) * 0.75);
        backLeftDrive.setPower(((vertical + horizontal)+pivot) * 0.75);
    }

    public void run(double verticalPosition, double horizontalPosition, double pivot){

        double y = -verticalPosition; // Remember, Y stick value is reversed
        double x = horizontalPosition; // Counteract imperfect strafing
        double rx = pivot;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y - x -rx) / denominator;
        double backLeftPower = (-y + x - rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    @Override
    public void runOpMode() {

    }

}
