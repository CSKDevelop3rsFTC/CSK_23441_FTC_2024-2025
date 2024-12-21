package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Camera Imports

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "lrAuto", group = "Autonomous")
public class lrAuto extends LinearOpMode {

    public class claw {
        private Servo clawServo;

        public claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setDirection(Servo.Direction.FORWARD);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.8);
                sleep(0);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawServo.setPosition(0.3);
                sleep(1000);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

    }

    public class slides {

        private DcMotor outtakeMotor1;
        private DcMotor outtakeMotor2;
        private Servo clawServo;

        public slides(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setDirection(Servo.Direction.FORWARD);
            outtakeMotor1 = hardwareMap.get(DcMotor.class, "outtakeMotor1");
            outtakeMotor2 = hardwareMap.get(DcMotor.class, "outtakeMotor2");
        }

        public class SlidesUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeMotor1.setTargetPosition(6000);
                outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor1.setPower(1);
                outtakeMotor2.setTargetPosition(6000);
                outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor2.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action slidesUp() {
            return new SlidesUp();
        }

        public class SlidesDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeMotor1.setTargetPosition(0);
                outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor1.setPower(1);
                outtakeMotor2.setTargetPosition(0);
                outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
                outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor2.setPower(1);
                sleep(650);
                return false;
            }
        }

        public Action slidesDown() {
            return new SlidesDown();
        }

        public class SlidesHold implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int currentPos = outtakeMotor1.getCurrentPosition();
                while (currentPos < 4000) {
                    sleep(10);
                    currentPos = outtakeMotor1.getCurrentPosition();
                }
                return false;
            }
        }

        public Action slidesHold() {
            return new SlidesHold();
        }

        public class SlidesMove implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int currentPos = -outtakeMotor1.getCurrentPosition();
                outtakeMotor1.setTargetPosition(1400);
                outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor1.setPower(1);
                outtakeMotor2.setTargetPosition(1400);
                outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor2.setPower(1);

                while (currentPos < 1390) {
                    sleep(1);
                    currentPos = -outtakeMotor1.getCurrentPosition();
                    System.out.println(currentPos);
                }

                outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                sleep(200);

                outtakeMotor1.setTargetPosition(0);
                outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
                outtakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor1.setPower(0.9);
                outtakeMotor2.setTargetPosition(0);
                outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
                outtakeMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeMotor2.setPower(0.9);

                while (currentPos > -50) {
                    sleep(1);
                    currentPos = outtakeMotor1.getCurrentPosition();
                    System.out.println(currentPos);
                }

                return false;
            }
        }

        public Action slidesMove() {
            return new SlidesMove();
        }

    }
    public class vFourBar {

        public Servo vFourbarServo1;
        public Servo vFourbarServo2;

        ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
        ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
        public vFourBar(HardwareMap hardwareMap) {
            vFourbarServo1 = hardwareMap.get(Servo.class, "vFourbarServo1");
            vFourbarServo2 = hardwareMap.get(Servo.class, "vFourbarServo2");
            vFourbarServo1.setDirection(Servo.Direction.FORWARD);
            vFourbarServo2.setDirection(Servo.Direction.REVERSE);
        }

        public class FourBarReset implements  Action {              // zero position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.13);
                sleep(350);
                vFourbarServo1.setPosition(0.0);
                return false;
            }
        }
        public Action fourBarReset(){
            return new FourBarReset();
        }

        public class FourBarHold implements  Action {              // hold position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.13);
                sleep(100);
                vFourbarServo1.setPosition(0.2);
                return false;
            }
        }
        public Action fourBarHold(){

            return new FourBarHold();
        }
        public class FourBarReady implements  Action {              // ready position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.4);
                sleep(500);
                vFourbarServo2.setPosition(0.7);
                return false;
            }
        }
        public Action fourBarReady(){

            return new FourBarReady();
        }

        public class FourBarDrop implements  Action {              // drop position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.96);
                return false;
            }
        }
        public Action fourBarDrop(){
            return new FourBarDrop();
        }

    }
    public class intake {
        public Servo hzFourbarServo1;
        public Servo hzFourbarServo2;
        ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
        ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
        public CRServo intakeServo;
        public intake(HardwareMap hardwareMap){
            hzFourbarServo1 = hardwareMap.get(Servo.class, "hzFourbarServo1");
            hzFourbarServo1.setDirection(Servo.Direction.REVERSE);
            hzFourbarServo2 = hardwareMap.get(Servo.class, "hzFourbarServo2");
            intakeServo    =  hardwareMap.get(CRServo.class, "intakeServo");
        }
        public void spinTake(double dir, int time){
            double intakeSpeed = 1;
            intakeServo.setPower(intakeSpeed * dir);
            sleep(time);
        }
        public void contSpin(boolean spinT){
            double speed = -1;
            if(spinT){
                intakeServo.setPower(speed);
            } else if(!spinT){
                intakeServo.setPower(0);
            }
        }
        public class IntakeSample implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
                hzFourbarServo1.setPosition(1);
                hzFourbarServo2.setPosition(1);
                spinTake(1,700);
                return false;
            }
        }
        public Action intakeSample(){
            return new IntakeSample();
        }
        public class RetractIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                contSpin(true);
                hzFourbarServo1.setPosition(0.10);
                hzFourbarServo2.setPosition(0.10);
                contSpin(false);
                return false;
            }
        }
        public Action retractIntake(){
            return new RetractIntake();
        }
        public class HoldIntake implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hzFourbarServo1.setPosition(0.10);
                hzFourbarServo2.setPosition(0.10);
                return false;
            }
        }
        public Action holdIntake(){
            return new HoldIntake();
        }

    }
    public class hold {
        public Servo    hzSlidesServo1;
        public Servo    hzSlidesServo2;
        ServoImplEx hzSlidesServo_1 = (ServoImplEx) hzSlidesServo1;
        ServoImplEx hzSlidesServo_2 = (ServoImplEx) hzSlidesServo2;
        public hold (HardwareMap hardwareMap){
            hzSlidesServo1    =      hardwareMap.get(Servo.class, "hzSlidesServo1");
            hzSlidesServo2    =      hardwareMap.get(Servo.class, "hzSlidesServo2");
            hzSlidesServo2.setDirection(Servo.Direction.REVERSE);
        }
        public class HoldIntakeSlides implements Action{

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hzSlidesServo_1.setPwmEnable();
                hzSlidesServo_2.setPwmEnable();
                return false;
            }
        }
        public Action holdIntakeSlides(){
            return new HoldIntakeSlides();
        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d beginPose = new Pose2d(-63, 36, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        claw clawServo = new claw(hardwareMap);
        slides outtakeMotor1 = new slides(hardwareMap);
        slides outtakeMotor2 = new slides(hardwareMap);
        vFourBar vFourbarServo1 = new vFourBar(hardwareMap);
        vFourBar vFourBarServo2 = new vFourBar(hardwareMap);
        intake hzFourbarServo1 = new intake(hardwareMap);
        intake hzFourbarServo2 = new intake(hardwareMap);
        intake intakeServo = new intake(hardwareMap);
        hold hzSlidesServo1 = new hold(hardwareMap);
        hold hzSlidesServo2 = new hold(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position
                .turnTo(Math.toRadians(130)); // turn around to the basket
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-56.9, 57, Math.toRadians(130)))
                .strafeTo(new Vector2d(-47, 49)) // move to sample one
                .turnTo(Math.toRadians(-180));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-47, 49, Math.toRadians(-180)))
                .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position
                .turnTo(Math.toRadians(130)); // turn around to the basket
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-56.9, 57, Math.toRadians(130)))
                .strafeTo(new Vector2d(-44.8, 61)) // move to sample position 2
                .turnTo(Math.toRadians(-180));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-44.8, 61, Math.toRadians(-180)))
                .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position
                .turnTo(Math.toRadians(130)); // turn around to the basket
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-56.9, 57, Math.toRadians(130)))
                .turnTo(Math.toRadians(0))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(0,24), Math.PI / 2); // move up to center rungs


        Action tab1Move = tab1.build(); // move to droppin pos for preloaded sample
        Action tab2Move = tab2.build(); // move to intake pos for sample one
        Action tab3Move = tab3.build(); // droppin pos for sample 1
        Action tab4Move = tab4.build(); // move to intake pos for sample two
        Action tab5Move = tab5.build(); // droppin pos for sample 2
        Action tab6Move = tab6.build(); // park under rungs

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(

                        //.setTangent(0)
                        //.strafeToConstantHeading(new Vector2d(-60,-57))
                        /**clawServo.openClaw(),
                         outtakeMotor1.slidesUp(),
                         outtakeMotor1.slidesHold(),
                         clawServo.closeClaw(),
                         outtakeMotor1.slidesHold(),
                         outtakeMotor1.slidesDown()**/
                        //outtakeMotor1.slidesMove()

                        hzSlidesServo1.holdIntakeSlides(),
                        tab1Move,

                        vFourbarServo1.fourBarReady(),
                        outtakeMotor1.slidesUp(),
                        vFourbarServo1.fourBarDrop(),
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarHold(),
                        outtakeMotor1.slidesDown()

                        /*

                        hzSlidesServo1.holdIntakeSlides(),
                        tab2Move,
                        hzFourbarServo1.intakeSample(),
                        hzFourbarServo1.retractIntake(),
                        vFourbarServo1.fourBarReset(),
                        clawServo.openClaw(),


                        tab3Move,
                        vFourbarServo1.fourBarReady(),
                        outtakeMotor1.slidesUp(),
                        vFourbarServo1.fourBarDrop(),
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarHold(),
                        outtakeMotor1.slidesDown(),


                        tab4Move,
                        hzFourbarServo1.intakeSample(),
                        hzFourbarServo1.retractIntake(),
                        vFourbarServo1.fourBarReset(),
                        clawServo.openClaw(),

                        tab5Move,
                        vFourbarServo1.fourBarReady(),
                        outtakeMotor1.slidesUp(),
                        vFourbarServo1.fourBarDrop(),
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarHold(),
                        outtakeMotor1.slidesDown(),

                        hzSlidesServo1.holdIntakeSlides(),
                        tab6Move

                         */


                        /**
                         .setTangent(0)
                         .splineToConstantHeading(new Vector2d(-41,0), Math.PI * 2) // move up to center rungs

                         .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen

                         .waitSeconds(2) // replace later with action of placing specimen

                         .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                         .strafeTo(new Vector2d(-47, 49)) // move to sample one

                         .turnTo(Math.toRadians(-180)) // do a 180 to have intake facing sample

                         .waitSeconds(2) // intake sample

                         .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position

                         .turnTo(Math.toRadians(130)) // turn around to the basket

                         .waitSeconds(2) // drop the pixels

                         .strafeTo(new Vector2d(-44.8, 61)) // move to sample position 2

                         .turnTo(Math.toRadians(-180))

                         .waitSeconds(2) // get the sample

                         .strafeTo(new Vector2d(-56.9, 57)) // move to dropping position

                         .turnTo(Math.toRadians(130)) // turn around to the basket

                         .waitSeconds(2)

                         .turnTo(Math.toRadians(0))

                         .setTangent(0)
                         .splineToConstantHeading(new Vector2d(0,-24), Math.PI / 2) // move up to center rungs

                         //.setTangent(Math.PI / 2)
                         //.setReversed(true)
                         //.splineToConstantHeading(new Vector2d(0, 0), Math.PI / 2)

                         **/

                )
        );

    }

}
