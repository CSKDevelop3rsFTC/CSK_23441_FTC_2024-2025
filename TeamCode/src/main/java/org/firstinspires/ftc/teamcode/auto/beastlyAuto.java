package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "beastlyAuto", group = "Autonomous")
public class beastlyAuto extends LinearOpMode {
    public class vFourBar {

        private Servo vFourbarServo1;
        private Servo vFourbarServo2;

        public vFourBar(HardwareMap hardwareMap) {
            vFourbarServo1 = hardwareMap.get(Servo.class, "vFourbarServo1");
            vFourbarServo2 = hardwareMap.get(Servo.class, "vFourbarServo2");
            vFourbarServo1.setDirection(Servo.Direction.REVERSE);
            vFourbarServo2.setDirection(Servo.Direction.REVERSE);
        }
        public class FourBarStart implements Action {              // grab from box position //pro was 0.13, 0.19

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.58); //.55
                vFourbarServo2.setPosition(0.5); //.53
                //vFourbarServo1.setPosition(0.67);
                // sleep(250);
                //  vFourbarServo2.setPosition(0.1);
                //sleep(500);
                return false;
            }
        }
        public Action fourBarStart(){
            return new FourBarStart();
        }

        public class FourBarStartEnter implements  Action {              // grab from box position //pro was 0.13, 0.19

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.48);
                vFourbarServo2.setPosition(0.67);
                return false;
            }
        }
        public Action fourBarStartEnter(){
            return new FourBarStartEnter();
        }

        public class FourBarReset implements  Action {              // hold position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.1);
                sleep(250);
                vFourbarServo1.setPosition(0.67);
                sleep(250);
                return false;
            }
        }
        public Action fourBarReset(){
            return new FourBarReset();
        }

        public class FourBarGrab implements  Action {              // grab from wall

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.23);
                sleep(350);
                vFourbarServo1.setPosition(0.94);
                sleep(550);
                return false;
            }
        }
        public Action fourBarGrab(){

            return new FourBarGrab();
        }

        public class FourBarUp implements  Action {              // lift from wall

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.57);
                sleep(500);
                return false;
            }
        }
        public Action fourBarUp(){

            return new FourBarUp();
        }
        public class FourBarPlace implements  Action {              // place on bar

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                // vFourbarServo1.setPosition(0.58);
                // sleep(350);
                vFourbarServo2.setPosition(0.62);
                //  sleep(350);
                return false;
            }
        }
        public Action fourBarPlace(){

            return new FourBarPlace();
        }

        public class FourBarEnter implements  Action {              // drop position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.64);
                //sleep(350);
                vFourbarServo2.setPosition(0.69);
                sleep(200);
                return false;
            }
        }
        public Action fourBarEnter(){
            return new FourBarEnter();
        }

    }
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
                clawServo.setPosition(0.44);
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
                clawServo.setPosition(0.27);
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

        public intake(HardwareMap hardwareMap) {
            hzFourbarServo1 = hardwareMap.get(Servo.class, "hzFourbarServo1");
            hzFourbarServo2 = hardwareMap.get(Servo.class, "hzFourbarServo2");
            hzSlidesServo1  = hardwareMap.get(Servo.class, "hzSlidesServo1");
            hzSlidesServo2  = hardwareMap.get(Servo.class, "hzSlidesServo2");
            hzSlidesServo2.setDirection(Servo.Direction.REVERSE);
            hzFourbarServo1.setDirection(Servo.Direction.REVERSE);
        }

        public class HzOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx hzSlidesServo_1 = (ServoImplEx) hzSlidesServo1;
                ServoImplEx hzSlidesServo_2 = (ServoImplEx) hzSlidesServo2;
                hzSlidesServo_1.setPwmEnable();
                hzSlidesServo_2.setPwmEnable();
                hzSlidesServo2.setPosition(0.50);
                hzSlidesServo1.setPosition(0.40);

                ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
                ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
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
                hzFourbarServo1.setPosition(0.13);
                hzFourbarServo2.setPosition(0.13);

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
                return false;
            }
        }

        public Action intakeMove() {
            return new Intake();
        }

    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-63, -11, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        claw clawServo = new claw(hardwareMap);
        intake hzFourbarServo1 = new intake(hardwareMap);
        intake hzSlidesServo1 = new intake(hardwareMap);
        intake hzFourbarServo2 = new intake(hardwareMap);
        intake hzSlidesServo2 = new intake(hardwareMap);
        vFourBar vFourbarServo1 = new vFourBar(hardwareMap);
        vFourBar vFourbarServo2 = new vFourBar(hardwareMap);

        TrajectoryActionBuilder poopee = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0,0));

        Action peepoo = poopee.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        peepoo
                )
        );


    }
}
