package org.firstinspires.ftc.teamcode.teleop;

//import statements

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MeetZero:Teleop", group="Robot")
public class   Meet0TeleOp extends LinearOpMode {

    //Drive Train Motors
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;

    //4 Stage Slides Motors (Outtake)
    public DcMotor  outtakeMotor1    = null;
    public DcMotor  outtakeMotor2    = null;

    //Intake Servos
    public Servo    hzFourbarServo1  = null;
    public Servo    hzFourbarServo2  = null;
    public Servo    hzSlidesServo1   = null;
    public Servo    hzSlidesServo2   = null;
    public CRServo  intakeServo      = null;

    //Outtake Servos
    public Servo    vFourbarServo1   = null;
    public Servo    vFourbarServo2   = null;
    public Servo    clawRotateServo  = null;
    public Servo    clawServo        = null;

    public double   speedDrive       = 0.65;
    public double   clawParam        = 0.3;
    public double   hzParam          = 0.1;
    public boolean  activeClaw       = false;
    public double   intakeSpeed      = 1;


    public void initialize(){
        frontLeftDrive    =    hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive   =   hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive     =     hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive    =    hardwareMap.get(DcMotor.class, "backRightDrive");

        outtakeMotor1     =     hardwareMap.get(DcMotor.class, "outtakeMotor1");
        outtakeMotor2     =     hardwareMap.get(DcMotor.class, "outtakeMotor2");

        hzFourbarServo1   =     hardwareMap.get(Servo.class, "hzFourbarServo1");
        hzFourbarServo2   =     hardwareMap.get(Servo.class, "hzFourbarServo2");
        hzSlidesServo1    =      hardwareMap.get(Servo.class, "hzSlidesServo1");
        hzSlidesServo2    =      hardwareMap.get(Servo.class, "hzSlidesServo2");
        intakeServo       =         hardwareMap.get(CRServo.class, "intakeServo");

        vFourbarServo1    =      hardwareMap.get(Servo.class, "vFourbarServo1");
        vFourbarServo2    =      hardwareMap.get(Servo.class, "vFourbarServo2");
        clawRotateServo   =     hardwareMap.get(Servo.class, "clawRotateServo");
        clawServo         =           hardwareMap.get(Servo.class, "clawServo");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

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

    public void spinTake() {
        if (gamepad1.left_bumper) {
            intakeServo.setPower(intakeSpeed);
        }

        else if (gamepad1.right_bumper) {
            intakeServo.setPower(-intakeSpeed);
        }
        else {
            intakeServo.setPower(0);
        }
    }

    public void driveController() {
        if (gamepad1.right_trigger > 0) {
            driveMovement(1);
        }

        else {
            driveMovement(speedDrive);
        }
    }

    public void clawOpen() {
        if (gamepad2.x) {
            if (!activeClaw){
                activeClaw = true;
                clawServo.setDirection(Servo.Direction.FORWARD);
                clawServo.setPosition(clawParam);
            }
        }
    }

    public void clawClose() {
        if (gamepad2.b) {
            if (activeClaw) {
                clawServo.setDirection(Servo.Direction.FORWARD);
                clawServo.setPosition(1 - clawParam);
                activeClaw = false;
            }
        }
    }


    public void reset(){ // when clicked A it will set the outatkeMoor 1 to 0
        if (Math.abs(outtakeMotor1.getCurrentPosition())>50){
            move(50, DcMotorSimple.Direction.FORWARD,0.95);
        }
    }

    public void reset2(){ // when clicked A it will set the outatkeMOtor 2 to 0
        if (Math.abs(outtakeMotor2.getCurrentPosition())>50){
            move(50, DcMotorSimple.Direction.REVERSE,0.95);
        }
    }

    private void move(int position, DcMotorSimple.Direction direction, double power){
        outtakeMotor1.setTargetPosition(position);
        outtakeMotor1.setDirection(direction);
        outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor1.setPower(power);
    }
    private void move2(int position, DcMotorSimple.Direction direction, double power){
        outtakeMotor2.setTargetPosition(position);
        outtakeMotor2.setDirection(direction);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeMotor2.setPower(power);
    }

    public void stopSlides(){
        if (!outtakeMotor1.isBusy() ) {
            outtakeMotor1.setPower(0);
        }
    }
    public void stopSlides2(){
        if (!outtakeMotor2.isBusy() ) {
            outtakeMotor2.setPower(0);
        }
    }

    public void lift(){  // brings outtake UP when RIGHT trigger held
        if (outtakeMotor1.getCurrentPosition()<6000) {
            move(outtakeMotor1.getCurrentPosition()+400, DcMotorSimple.Direction.REVERSE,0.7);
        }

    }
    public void lift2(){  // brings outtake UP when RIGHT trigger held
        if (outtakeMotor2.getCurrentPosition()<6000) {
            move(outtakeMotor2.getCurrentPosition()+400, DcMotorSimple.Direction.FORWARD,0.7);
        }

    }
    public void drop() {  // brings outtake DOWN when LEFT trigger held
        if (Math.abs(outtakeMotor1.getCurrentPosition()) > 60) {
            move(-(Math.abs(outtakeMotor1.getCurrentPosition()) - 100), DcMotorSimple.Direction.FORWARD, 0.5);
        }
    }
    public void drop2() {  // brings outtake DOWN when LEFT trigger held
        if (Math.abs(outtakeMotor2.getCurrentPosition()) > 60) {
            move(-(Math.abs(outtakeMotor2.getCurrentPosition()) - 100), DcMotorSimple.Direction.REVERSE, 0.5);
        }
    }


    public void slideMovement() {
        if (gamepad2.left_trigger > 0) {
            drop();
            drop2();
            System.out.println(outtakeMotor1.getCurrentPosition());
            System.out.println(outtakeMotor2.getCurrentPosition());
        }

        else if (gamepad2.right_trigger > 0) {
            lift();
            lift2();
            System.out.println(outtakeMotor1.getCurrentPosition());
            System.out.println(outtakeMotor2.getCurrentPosition());

        }
        else if (gamepad2.a) {
            reset();
            reset2();
            System.out.println(outtakeMotor1.getCurrentPosition());
            System.out.println(outtakeMotor2.getCurrentPosition());

        }

        else {
            stopSlides();
        }
    }

    public void hzMovement() {

        hzSlidesServo2.setDirection(Servo.Direction.REVERSE);
        if (gamepad1.right_trigger > 0) {
            hzSlidesServo2.setPosition(0.4);
            hzSlidesServo1.setPosition(0.3);
        }
        if (gamepad1.left_trigger > 0) {
            hzSlidesServo1.setPosition(0.9);
            hzSlidesServo2.setPosition(1);
        }
    }

    public void hzFourBar() {
        hzFourbarServo2.setDirection(Servo.Direction.REVERSE);
        if (gamepad1.right_bumper) {
            hzFourbarServo1.setPosition(0.1);
            hzFourbarServo2.setPosition(0.1);
        }
        if (gamepad1.left_bumper) {
            hzFourbarServo1.setPosition(0.2);
            hzFourbarServo2.setPosition(0.2);
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

            clawOpen();

            clawClose();

            spinTake();

            slideMovement();

            //hzMovement();

            //hzFourBar();

            //slideMovement();

            //hzPowerMovement();
             sleep(50);

        }

    }

}
