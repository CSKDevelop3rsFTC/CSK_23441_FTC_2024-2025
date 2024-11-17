package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
    public Servo hzFourbarServo1 = null;
    public Servo hzFourbarServo2 = null;
    public Servo hzSlidesServo1 = null;
    public Servo hzSlidesServo2 = null;
    public CRServo intakeServo = null;

    //Outtake Servos
    public Servo vFourbarServo1 = null;
    public Servo vFourbarServo2 = null;
    public Servo clawRotateServo = null;
    public Servo clawServo = null;

    public double driveSpeed = 0.65;
    public double intakeSpeed = 1;
    public double clawParam   = 0.3;



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
    public void spinTake(double dir, int time){
        intakeServo.setPower(intakeSpeed * dir);
        sleep(time);
    }
    public void hzFourBar(int pos){
        if(pos == 1){
            hzFourbarServo1.setPosition(0.95);
            hzFourbarServo2.setPosition(0.95);
        } else {
            hzFourbarServo1.setPosition(0.1);
            hzFourbarServo2.setPosition(0.1);
        }
    }
    public void intakeSample(){
        hzFourBar(1);
        spinTake(1,300);
        hzFourBar(0);
        spinTake(-1,100);
    }

    public void clawPosition(int pos){ // 0 = close 1 = open
        if(pos == 1) {
            clawServo.setDirection(Servo.Direction.FORWARD);
            clawServo.setPosition(clawParam);
        } else if(pos == 0) {
            clawServo.setDirection(Servo.Direction.FORWARD);
            clawServo.setPosition(1 - clawParam);
        }
    }
    public void move(int position, DcMotorSimple.Direction direction, double power){
        outtakeMotor1.setTargetPosition(position);
        outtakeMotor1.setDirection(direction);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor1.setPower(power);
    }
    public void move2(int position, DcMotorSimple.Direction direction, double power){
        outtakeMotor2.setTargetPosition(position);
        outtakeMotor2.setDirection(direction);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor2.setPower(power);
    }

    public void lift(){  // brings outtake UP
        if (outtakeMotor1.getCurrentPosition()<6000) {
            move(outtakeMotor1.getCurrentPosition()+300, DcMotorSimple.Direction.REVERSE,1);
        }

    }
    public void lift2() {  // brings outtake UP
        if (outtakeMotor2.getCurrentPosition() < 6000) {
            move2(outtakeMotor2.getCurrentPosition() + 300, DcMotorSimple.Direction.FORWARD, 1);
        }
    }
    public void slidesFourBar(int pos){   // pos == 1 == lift // pos 2 = hold
        ServoImplEx vFourbarServo_1 = (ServoImplEx) hzSlidesServo1;
        ServoImplEx vFourbarServo_2 = (ServoImplEx) hzSlidesServo2;
        if(pos == 0) {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo_1.setPosition(0);
            sleep(350);
            vFourbarServo_2.setPosition(0);
        } else if(pos == 1) {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo_2.setPosition(1);
            vFourbarServo_1.setPosition(0.4);
        } else if(pos == 2){
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo2.setPosition(0.5);
            sleep(100);
            vFourbarServo1.setPosition(0.25);
        }

    }
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        outtakeMotor1 = hardwareMap.get(DcMotor.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "outtakeMotor2");

        hzFourbarServo1 = hardwareMap.get(Servo.class, "hzfourbarServo1");
        hzFourbarServo2 = hardwareMap.get(Servo.class, "hzfourbarServo2");
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

        intakeSample();

        stopRobot();

    }



}
// hello this is aahan