package org.firstinspires.ftc.teamcode.teleop;

//import statements--

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name="MeetTwo:Red", group="Robot")
public class MeetTwoTeleOpRed extends LinearOpMode {

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
    public Servo    specClaw        = null;
    public CRServo  intakeServo      = null;

    //Outtake Servos
    public Servo    vFourbarServo1   = null;
    public Servo    vFourbarServo2   = null;
    public Servo    clawRotateServo  = null;
    public Servo    clawServo        = null;

    public ColorSensor cSense        = null;

    public double   speedDrive       = 0.65;
    public double   clawParam        = 0;
    public boolean  outPos1          = false;
    public boolean  activeClaw       = false;
    public double   intakeSpeed      = -1;
    public String   cTest            = "";

    public int redVal = 0;
    public int greenVal = 0;
    public int blueVal = 0;

    public int motor1pos = 0;
    public int motor2pos = 0;

    public int smotor1pos = 0;
    public int smotor2pos = 0;

    public boolean vInit = false;

    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamePad1 = new Gamepad();


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
        specClaw          =             hardwareMap.get(Servo.class, "specServo");

        cSense            =        hardwareMap.get(ColorSensor.class, "cSense");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        hzSlidesServo2.setDirection(Servo.Direction.REVERSE);
        hzFourbarServo1.setDirection(Servo.Direction.REVERSE);

        vFourbarServo1.setDirection(Servo.Direction.REVERSE);
        vFourbarServo2.setDirection(Servo.Direction.REVERSE);

        cSense.enableLed(true);

        activeClaw = false;

        smotor1pos = (outtakeMotor1.getCurrentPosition());
        smotor2pos = (outtakeMotor2.getCurrentPosition());



        outtakeMotor2.setTargetPosition(0);
        outtakeMotor2.setTargetPosition(0);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void driveMovement(double driveSpeed){
        double vertical;
        double horizontal;
        double pivot;

        horizontal =    gamepad1.left_stick_x;
        vertical   =   -gamepad1.left_stick_y;
        pivot      =   -gamepad1.right_stick_x * 0.5;

        frontRightDrive.setPower(((vertical + horizontal)-pivot) * driveSpeed);
        frontLeftDrive.setPower(((vertical - horizontal)+pivot) * driveSpeed);
        backRightDrive.setPower(((vertical - horizontal)-pivot) * driveSpeed);
        backLeftDrive.setPower(((vertical + horizontal)+pivot) * driveSpeed);
    }

    public void spinTake() {
        if (gamepad1.left_trigger > 0) {
            intakeServo.setPower(-intakeSpeed);
        }

        else if (gamepad1.right_trigger > 0) {
            intakeServo.setPower(intakeSpeed);
        }

        else {
            intakeServo.setPower(0);
        }
    }

    public void contSpin(boolean spinT) {
        if (spinT) {
            intakeServo.setPower(intakeSpeed);
        }
        else if (!spinT) {
            intakeServo.setPower(0);
        }
    }

    public void driveController() {
        if (gamepad1.y) {
            driveMovement(1);
        }

        else {
            driveMovement(speedDrive);
        }
    }

    public void clawClose() {
        ServoImplEx clawServ = (ServoImplEx) clawServo;
        clawServ.setPwmEnable();
        if (gamepad2.x) {
            if (activeClaw){
                activeClaw = false;
                clawServo.setDirection(Servo.Direction.FORWARD);
                clawServo.setPosition(0.44);
            }
        }
    }


    public void clawOpen() {
        ServoImplEx clawServ = (ServoImplEx) clawServo;
        clawServ.setPwmEnable();
        if (gamepad2.b) {
            if (!activeClaw) {
                clawServo.setDirection(Servo.Direction.FORWARD);
                clawServo.setPosition(0.27);
                activeClaw = true;
            }
        }
    }




    public void reset(){ // when clicked A it will set the outtakeMotor 1 to 0
        motor1pos = Math.abs(outtakeMotor1.getCurrentPosition());
        if (Math.abs(motor1pos) > (40 + Math.abs(smotor1pos))) {
            move(smotor1pos - 40, DcMotorSimple.Direction.FORWARD, 0.2);
        }
    }

    public void reset2(){ // when clicked A it will set the outtakeMotor 2 to 0
        motor2pos = Math.abs(outtakeMotor2.getCurrentPosition());
        if (motor2pos > (40 + Math.abs(smotor2pos))){
            move2(-smotor2pos - 40, DcMotorSimple.Direction.REVERSE,0.2);
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
        if (!outtakeMotor1.isBusy()) {
            outtakeMotor1.setPower(0);
        }
    }
    public void stopSlides2(){
        if (!outtakeMotor2.isBusy() ) {
            outtakeMotor2.setPower(0);
        }
    }

    public void lift(){  // brings outtake UP when RIGHT trigger held
        motor1pos = outtakeMotor1.getCurrentPosition();
        if (Math.abs(motor1pos) < 6000 + Math.abs(motor1pos)) {
            move(motor1pos+300, DcMotorSimple.Direction.FORWARD,0.2);
        }

    }
    public void lift2(){  // brings outtake UP when RIGHT trigger held
        motor2pos = outtakeMotor2.getCurrentPosition();
        if (Math.abs(motor2pos) < 6000 + Math.abs(motor2pos)) {
            move2(motor2pos+300, DcMotorSimple.Direction.REVERSE,0.2);
        }

    }
    public void drop() {  // brings outtake DOWN when LEFT trigger held
        if (Math.abs(outtakeMotor1.getCurrentPosition()) > 80) {
            move(-(Math.abs(outtakeMotor1.getCurrentPosition()) - 30), DcMotorSimple.Direction.FORWARD, 0.1);
        }
    }
    public void drop2() {  // brings outtake DOWN when LEFT trigger held
        if (Math.abs(outtakeMotor2.getCurrentPosition()) > 80) {
            move2(-(Math.abs(outtakeMotor2.getCurrentPosition()) - 300), DcMotorSimple.Direction.REVERSE, 0.1);
        }
    }


    public void slideMovement() {
        if (gamepad2.left_trigger > 0) {
            //drop();
            //drop2();
        }

        else if (gamepad2.right_trigger > 0) {
            //lift();
            //lift2();

        }
        else if (gamepad2.a) {
            vSlides("hold");
            //reset();
            //reset2();

        }
        /**
         else {
         stopSlides();
         stopSlides2();

         }
         **/
    }

    public void vSlides(String vPos) {
        ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
        ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;

        if (vPos == "zero") {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo2.setPosition(0.0);
            vFourbarServo1.setPosition(0.8);
            sleep(300);
            clawServo.setPosition(0.27);
            activeClaw = true;
        }

        else if (vPos == "enter") {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo1.setPosition(0.64);
            sleep(350);
            vFourbarServo2.setPosition(0.69);
        }

        else if (vPos == "grab") {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            //vFourbarServo2.setPosition(0.23);
            vFourbarServo2.setPosition(0.5);
            sleep(350);
            //vFourbarServo1.setPosition(0.86);
            vFourbarServo1.setPosition(0.45);
        }
        else if (vPos == "up"){
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo2.setPosition(0.21);
            sleep(350);
        }
        else if (vPos == "place"){
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmDisable();
            // vFourbarServo1.setPosition(0.67); previously 0.49
            // sleep(350);
            vFourbarServo2.setPosition(0.62);

        }

        else if (vPos == "ready") {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo1.setPosition(0.67);
            sleep(200);
            vFourbarServo2.setPosition(0.5);
            sleep(250);
            vFourbarServo1.setPosition(0.57);
        }

        else if (vPos == "hold") {
            vFourbarServo_1.setPwmEnable();
            vFourbarServo_2.setPwmEnable();
            vFourbarServo2.setPosition(0.0);
            sleep(350);
            vFourbarServo1.setPosition(0.67);
        }

        else if (vPos == "drop") {
            vFourbarServo_2.setPwmEnable();
            vFourbarServo2.setPosition(0.95);
            sleep(200);
            vFourbarServo1.setPosition(0.25);
            sleep(300);
            clawServo.setPosition(0.44);
            activeClaw = false;
        }



    }

    public void colorSense() {
        sleep(300);

        greenVal = cSense.green();
        redVal = cSense.red();
        blueVal = cSense.blue();

        System.out.println("R : " + redVal);
        System.out.println("G : " + greenVal);
        System.out.println("B : " + blueVal);

        if (redVal > blueVal && redVal > greenVal) {
            vSlides("zero");
            sleep(350);
            clawServo.setDirection(Servo.Direction.FORWARD);
            clawServo.setPosition(0.27);
            activeClaw = true;
            sleep(150);
            vSlides("ready");

            cTest = "red";
        }
        else if (blueVal > redVal && blueVal > greenVal) {
            vSlides("zero");
            sleep(250);
            clawServo.setDirection(Servo.Direction.FORWARD);
            clawServo.setPosition(0.27);
            activeClaw = true;
            sleep(200);
            vSlides("ready");
            sleep(150);
            vSlides("drop");
            sleep(150);
            vSlides("hold");

            cTest = "blue";
        }
        else if (redVal > blueVal && greenVal > blueVal) {

            vSlides("zero");
            sleep(150);
            vSlides("ready");

            cTest = "yellow";
        }

    }

    public void slidesFourBar() {


        if (gamepad2.y) {
            vSlides("grab");
        } else if (gamepad2.right_bumper) {
            vSlides("enter");
        } else if (gamepad2.left_bumper) {
            vSlides("place");
        }


        if (gamepad1.x) {
            vSlides("drop");
        }

        else if (gamepad1.y) {

        }
        else if (gamepad1.b) {

        }



    }



    public void hzMovement() {

        ServoImplEx hzSlidesServo_1 = (ServoImplEx) hzSlidesServo1;
        ServoImplEx hzSlidesServo_2 = (ServoImplEx) hzSlidesServo2;

        if ((!currentGamepad1.right_bumper && previousGamePad1.right_bumper) && outPos1 == false) {

            hzSlidesServo_1.setPwmEnable();
            hzSlidesServo_2.setPwmEnable();
            hzSlidesServo2.setPosition(0.7);
            hzSlidesServo1.setPosition(0.6);

            sleep(400);
            outPos1 = true;
        }

        else if ((!currentGamepad1.right_bumper && previousGamePad1.right_bumper) && outPos1 == true) {

            hzSlidesServo_1.setPwmEnable();
            hzSlidesServo_2.setPwmEnable();
            hzSlidesServo2.setPosition(0.45);
            hzSlidesServo1.setPosition(0.35);

        }

        if (gamepad1.left_bumper) {
            contSpin(true);
            hzSlidesServo_1.setPwmEnable();
            hzSlidesServo_2.setPwmEnable();
            hzFourbarServo1.setPosition(0.13);
            hzFourbarServo2.setPosition(0.13);
            sleep(50);
            hzSlidesServo2.setPosition(0.94);
            hzSlidesServo1.setPosition(0.84);
            sleep(500);
            contSpin(false);
            outPos1 = false;
            colorSense();
        }
    }

    public void hzFourBar() {
        ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
        ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;

        if (gamepad1.a) {
            hzFourBarServo_1.setPwmEnable();
            hzFourBarServo_2.setPwmEnable();
            hzFourbarServo1.setPosition(1);
            hzFourbarServo2.setPosition(1);
        }


    }

    public void reGamepad() {
        previousGamePad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
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

            reGamepad();
            /*
            System.out.println("R : " + cSense.red());
            System.out.println("G : " + cSense.green());
            System.out.println("B : " + cSense.blue());
            */

            System.out.println("Color : " + cTest);



            System.out.println("motor1:" + outtakeMotor1.getCurrentPosition() + "motor2:" + outtakeMotor2.getCurrentPosition());

            System.out.println("start 1 : " + smotor1pos + " start 2 : " + smotor2pos);

            System.out.println("start 1 target : " + outtakeMotor1.getTargetPosition() + " start 2 target: " + outtakeMotor2.getTargetPosition());



            driveController();

            clawOpen();

            clawClose();

            spinTake();

            slideMovement();

            slidesFourBar();

            hzMovement();

            hzFourBar();

            sleep(50);

        }

    }

}

// new git token push