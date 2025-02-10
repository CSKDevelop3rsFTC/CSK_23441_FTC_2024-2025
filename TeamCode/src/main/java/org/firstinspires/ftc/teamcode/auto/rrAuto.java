package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Camera Imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.WebcamExample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;
import java.util.Vector;

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

@Autonomous(name = "rrAuto", group = "Autonomous")
public class rrAuto extends LinearOpMode {

    public class claw {
        private Servo clawServo;

        public claw(HardwareMap hardwareMap) {
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setDirection(Servo.Direction.FORWARD);
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //ServoImplEx claw_Servo = (ServoImplEx) clawServo;
                //claw_Servo.setPwmEnable();
                clawServo.setPosition(0.54);
                sleep(300);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //ServoImplEx claw_Servo = (ServoImplEx) clawServo;
                //claw_Servo.setPwmEnable();
                clawServo.setPosition(0.47);
                sleep(300);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

    }

    public class intake {
        private Servo hzFourbarServo1;
        private Servo hzFourbarServo2;
        private Servo hzSlidesServo1;
        private Servo hzSlidesServo2;
        public CRServo intakeServo;

        public intake(HardwareMap hardwareMap) {
            hzFourbarServo1 = hardwareMap.get(Servo.class, "hzFourbarServo1");
            hzFourbarServo2 = hardwareMap.get(Servo.class, "hzFourbarServo2");
            hzSlidesServo1  = hardwareMap.get(Servo.class, "hzSlidesServo1");
            hzSlidesServo2  = hardwareMap.get(Servo.class, "hzSlidesServo2");
            intakeServo     = hardwareMap.get(CRServo.class, "intakeServo");
            hzSlidesServo2.setDirection(Servo.Direction.REVERSE);
            hzFourbarServo1.setDirection(Servo.Direction.REVERSE);
        }

        public class HzOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                /*
                ServoImplEx hzSlidesServo_1 = (ServoImplEx) hzSlidesServo1;
                ServoImplEx hzSlidesServo_2 = (ServoImplEx) hzSlidesServo2;
                hzSlidesServo_1.setPwmEnable();
                hzSlidesServo_2.setPwmEnable();
                hzSlidesServo2.setPosition(0.50);
                hzSlidesServo1.setPosition(0.40);

                 */

                ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
                ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
                intakeServo.setPower(-1);
                hzFourbarServo1.setPosition(0.98);
                hzFourbarServo2.setPosition(0.98);

                return false;
            }
        }
        public Action hzOut() {
            return new HzOut();
        }

        public class HzReset implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
                ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
                hzFourbarServo1.setPosition(0.18);
                hzFourbarServo2.setPosition(0.18);

                return false;
            }
        }
        public Action hzReset() {
            return new HzReset();
        }


        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ServoImplEx hzSlidesServo_1 = (ServoImplEx) hzSlidesServo1;
                ServoImplEx hzSlidesServo_2 = (ServoImplEx) hzSlidesServo2;
                hzSlidesServo_1.setPwmEnable();
                hzSlidesServo_2.setPwmEnable();
                hzSlidesServo2.setPosition(0.99);
                hzSlidesServo1.setPosition(0.89);

                ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
                ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
                hzFourbarServo1.setPosition(0.13);
                hzFourbarServo2.setPosition(0.13);

                intakeServo.setPower(0);
                return false;
            }
        }

        public Action intakeMove() {
            return new Intake();
        }

    }
    public class vFourBar {

        private Servo vFourbarServo1;
        private Servo vFourbarServo2;
        private Servo clawServo;

        public vFourBar(HardwareMap hardwareMap) {
            vFourbarServo1 = hardwareMap.get(Servo.class, "vFourbarServo1");
            vFourbarServo2 = hardwareMap.get(Servo.class, "vFourbarServo2");
            clawServo = hardwareMap.get(Servo.class, "clawServo");
            clawServo.setDirection(Servo.Direction.FORWARD);
            vFourbarServo1.setDirection(Servo.Direction.REVERSE);
            vFourbarServo2.setDirection(Servo.Direction.REVERSE);
        }

        public class FourBarReset implements  Action {              // hold position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.67); //0.67
                vFourbarServo2.setPosition(0.0);
                return false;
            }
        }
        public Action fourBarReset(){
            return new FourBarReset();
        }

        public class FourBarStart implements  Action {              // hold position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.67); //0.67
                sleep(300);
                vFourbarServo2.setPosition(0.0);
                sleep(300);
                return false;
            }
        }
        public Action fourBarStart(){
            return new FourBarReset();
        }

        public class FourBarGrab implements  Action {              // grab from wall

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.95);
                vFourbarServo1.setPosition(0.25);

                return false;
            }
        }
        public Action fourBarGrab(){

            return new FourBarGrab();
        }

        public class FourBarPlace implements  Action {              // place on bar

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmDisable();
                vFourbarServo2.setPosition(0.0);
                vFourbarServo1.setPosition(0.81);
                sleep(250);
                clawServo.setPosition(0.47);
                sleep(100);
                vFourbarServo1.setPosition(0.6);
                vFourbarServo2.setPosition(0.55);
                sleep(100);
                clawServo.setPosition(0.54);
                return false;
            }
        }
        public Action fourBarPlace(){

            return new FourBarPlace();
        }

    }

    public class slides {

        private DcMotor outtakeMotor1;
        private DcMotor outtakeMotor2;
        private Servo specServo;

        public slides(HardwareMap hardwareMap) {
            specServo = hardwareMap.get(Servo.class, "specServo");
            specServo.setDirection(Servo.Direction.FORWARD);
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

        public class SlidesMoveUp implements Action {
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

                return false;
            }

            public Action slidesMoveUp() {
                return new SlidesMoveUp();
            }

            public class SlidesMoveDown implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    int currentPos = -outtakeMotor1.getCurrentPosition();

                    outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            public Action slidesMoveDown() {
                return new SlidesMoveDown();
            }

        }
    }

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d beginPose = new Pose2d(-64, 34, Math.toRadians(0));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        claw clawServo = new claw(hardwareMap);
        slides outtakeMotor1 = new slides(hardwareMap);
        slides outtakeMotor2 = new slides(hardwareMap);
        intake hzFourbarServo1 = new intake(hardwareMap);
        intake hzSlidesServo1 = new intake(hardwareMap);
        intake hzFourbarServo2 = new intake(hardwareMap);
        intake hzSlidesServo2 = new intake(hardwareMap);
        vFourBar vFourbarServo1 = new vFourBar(hardwareMap);
        vFourBar vFourbarServo2 = new vFourBar(hardwareMap);

        TrajectoryActionBuilder turnTest = drive.actionBuilder(beginPose)
                .turnTo(Math.toRadians(40))
                .waitSeconds(1)
                .turnTo(Math.toRadians(80))
                .waitSeconds(1)
                .turnTo(Math.toRadians(120))
                .waitSeconds(1)
                .turnTo(Math.toRadians(160))
                .waitSeconds(1)
                .turnTo(Math.toRadians(200))
                .waitSeconds(1)
                .turnTo(Math.toRadians(240))
                .waitSeconds(1)
                .turnTo(Math.toRadians(280))
                .waitSeconds(1)
                .turnTo(Math.toRadians(320))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1);



        TrajectoryActionBuilder preload = drive.actionBuilder(beginPose)
                //.afterTime(0, vFourbarServo1.fourBarPlace())
                .splineToSplineHeading(new Pose2d(-57, 60, Math.toRadians(-40)), Math.PI)

                .waitSeconds(1);

        TrajectoryActionBuilder sample1 = drive.actionBuilder(new Pose2d(-57, 60, Math.toRadians(-40)))
               // .afterTime(0, vFourbarServo1.fourBarGrab())
                .strafeToSplineHeading(new Vector2d(-37, 49), Math.toRadians(0))

                .waitSeconds(1);

        TrajectoryActionBuilder place1 = drive.actionBuilder(new Pose2d(-37, 49, Math.toRadians(0)))
               // .afterTime(0, vFourbarServo1.fourBarPlace())
                .strafeToSplineHeading(new Vector2d(-57, 60), Math.toRadians(-40))

                .waitSeconds(1);

        TrajectoryActionBuilder sample2 = drive.actionBuilder(new Pose2d(-57, 60, Math.toRadians(-40)))
                //.afterTime(0, vFourbarServo1.fourBarGrab())
                .strafeToSplineHeading(new Vector2d(-37, 59), Math.toRadians(0))

                .waitSeconds(1);

        TrajectoryActionBuilder place2 = drive.actionBuilder(new Pose2d(-37, 59, Math.toRadians(0)))
                //.afterTime(0, vFourbarServo1.fourBarPlace())
                .strafeToSplineHeading(new Vector2d(-57, 60), Math.toRadians(-40))

                .waitSeconds(1);

        TrajectoryActionBuilder sample3 = drive.actionBuilder(new Pose2d(-57, 60, Math.toRadians(-40)))
                //.afterTime(0, vFourbarServo1.fourBarGrab())
                .strafeToSplineHeading(new Vector2d(-33.5, 62), Math.toRadians(40))

                .waitSeconds(1);

        TrajectoryActionBuilder place3 = drive.actionBuilder(new Pose2d(-33.5, 62, Math.toRadians(40)))
                //.afterTime(0, vFourbarServo1.fourBarPlace())
                .strafeToSplineHeading(new Vector2d(-57, 60), Math.toRadians(-40))

                .waitSeconds(1);

        Action turning = turnTest.build();

        Action preloadMove = preload.build();

        Action sample1Move = sample1.build();

        Action place1Move= place1.build();

        Action sample2Move = sample2.build();

        Action place2Move= place2.build();

        Action sample3Move = sample3.build();

        Action place3Move= place3.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //clawServo.closeClaw(),
                        //hzFourbarServo1.hzReset(),
                        //vFourbarServo1.fourBarStart(),
                        preloadMove,
                        //clawServo.closeClaw(),
                        sample1Move,
                        place1Move,
                        //clawServo.closeClaw(),
                        sample2Move,
                        place2Move,
                        //clawServo.closeClaw(),
                        sample3Move,
                        place3Move
                        //clawServo.closeClaw()



                        )
        );

    }

}