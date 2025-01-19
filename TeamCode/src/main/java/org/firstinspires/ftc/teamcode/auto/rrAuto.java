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
                clawServo.setPosition(0.44);
                sleep(450);
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
                sleep(450);
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
        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hzSlidesServo2.setDirection(Servo.Direction.REVERSE);
                hzFourbarServo1.setDirection(Servo.Direction.REVERSE);
                ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
                ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
                hzFourbarServo1.setPosition(0.13);
                hzFourbarServo2.setPosition(0.13);
                ServoImplEx hzSlidesServo_1 = (ServoImplEx) hzSlidesServo1;
                ServoImplEx hzSlidesServo_2 = (ServoImplEx) hzSlidesServo2;
                hzSlidesServo_1.setPwmEnable();
                hzSlidesServo_2.setPwmEnable();
                hzSlidesServo2.setPosition(0.99);
                hzSlidesServo1.setPosition(0.89);
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

        public vFourBar(HardwareMap hardwareMap) {
            vFourbarServo1 = hardwareMap.get(Servo.class, "vFourbarServo1");
            vFourbarServo2 = hardwareMap.get(Servo.class, "vFourbarServo2");
            vFourbarServo1.setDirection(Servo.Direction.REVERSE);
            vFourbarServo2.setDirection(Servo.Direction.REVERSE);
        }
        public class FourBarStart implements  Action {              // grab from box position //pro was 0.13, 0.19

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.12);
                sleep(350);
                vFourbarServo1.setPosition(0.21);
                sleep(350);
                return false;
            }
        }
        public Action fourBarStart(){
            return new FourBarStart();
        }

        public class FourBarReset implements  Action {              // hold position

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ServoImplEx vFourbarServo_1 = (ServoImplEx) vFourbarServo1;
                ServoImplEx vFourbarServo_2 = (ServoImplEx) vFourbarServo2;
                vFourbarServo_1.setPwmEnable();
                vFourbarServo_2.setPwmEnable();
                vFourbarServo2.setPosition(0.1);
                sleep(350);
                vFourbarServo1.setPosition(0.67);
                sleep(400);
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
                vFourbarServo2.setPosition(0.2);
                sleep(350);
                vFourbarServo1.setPosition(0.98);
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
                vFourbarServo1.setPosition(0.6);
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
                vFourbarServo1.setPosition(0.55);
                sleep(350);
                vFourbarServo2.setPosition(0.46);
                sleep(350);
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
                vFourbarServo_2.setPwmEnable();
                vFourbarServo1.setPosition(0.51);
                sleep(350);
                vFourbarServo2.setPosition(0.79);
                sleep(700);
                return false;
            }
        }
        public Action fourBarEnter(){
            return new FourBarEnter();
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
        Pose2d beginPose = new Pose2d(-63, -11, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        claw clawServo = new claw(hardwareMap);
        slides outtakeMotor1 = new slides(hardwareMap);
        slides outtakeMotor2 = new slides(hardwareMap);
        intake hzFourbarServo1 = new intake(hardwareMap);
        vFourBar vFourbarServo1 = new vFourBar(hardwareMap);
        vFourBar vFourbarServo2 = new vFourBar(hardwareMap);

        TrajectoryActionBuilder preLoad1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-45, -7)); // go forward
                // move Vfourbar to enter pos
        TrajectoryActionBuilder preload2 = drive.actionBuilder(new Pose2d(-45,-7,0))
                .strafeTo(new Vector2d(-32, -7));
                //place specimen
        TrajectoryActionBuilder preLoad3 = drive.actionBuilder(new Pose2d(-32, -7,0))
                .strafeTo(new Vector2d(-40,-7)); //move back and place specimen
        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(-40,-7,0))
                .strafeTo(new Vector2d(-55,0)) // backwards
                .strafeTo(new Vector2d(-50,-30))

                .strafeTo(new Vector2d(-10,-46)) // infront
                .strafeTo(new Vector2d(-45,-46)) // sample 1 pushed

                //.strafeTo(new Vector2d(-10,-46))
                .strafeTo(new Vector2d(-10,-54)) // infront 2
                .strafeTo(new Vector2d(-45,-54)) // sample 2 pushed

                //.strafeTo(new Vector2d(-10,-54))
                .strafeTo(new Vector2d(-10,-61.5))
                .strafeTo(new Vector2d(-55,-61.5))// infront 3
                .strafeTo(new Vector2d(-59.5,-56)); // sample 3 pushed

                // perform icy grab

        TrajectoryActionBuilder unHook1 = drive.actionBuilder(new Pose2d(-59.5,-56,0))
                .strafeTo(new Vector2d(-63 , -55.5)); // go forward to  enter pos

        TrajectoryActionBuilder moveUp = drive.actionBuilder(new Pose2d(-63,-55.5,0))
                .strafeTo(new Vector2d(-45 , -3)); // go forward to  enter pos

        TrajectoryActionBuilder lineUp = drive.actionBuilder(new Pose2d(-45,-3,0))
                .strafeTo(new Vector2d(-32, -3)); // lineup to place
                //place specimen

        TrajectoryActionBuilder reset = drive.actionBuilder(new Pose2d(-32, -3,0))
                .strafeTo(new Vector2d(-40,-3)); // move backwards then release specimen

        TrajectoryActionBuilder get1 = drive.actionBuilder(new Pose2d(-40,-3,0))
                .strafeTo(new Vector2d(-59,-25));

        TrajectoryActionBuilder unHook2 = drive.actionBuilder(new Pose2d(-59,-25,0))
                .strafeTo(new Vector2d(-63 , -25)); // go forward to  enter pos

        TrajectoryActionBuilder enterPos = drive.actionBuilder(new Pose2d(-63,-25,0))
                .strafeTo(new Vector2d(-45 , -3)); // go forward to  enter pos
                //use this command multiple times after each get

        TrajectoryActionBuilder lineUp1 = drive.actionBuilder(new Pose2d(-45,-3,0))
                .strafeTo(new Vector2d(-32, -2)); // go forward to line up specimen
                // replace later with action of placing specimen
        TrajectoryActionBuilder reset1 = drive.actionBuilder(new Pose2d(-32, -2,0))
                .strafeTo(new Vector2d(-40,-2));

        TrajectoryActionBuilder get2 = drive.actionBuilder(new Pose2d(-40,-2,0))
                .strafeTo(new Vector2d(-59,-25));

        TrajectoryActionBuilder lineUp2 = drive.actionBuilder(new Pose2d(-45,-3,0))
                .strafeTo(new Vector2d(-32, 4)); // go forward to line up specimen

                 // replace later with action of placing specimen
        TrajectoryActionBuilder reset2 = drive.actionBuilder(new Pose2d(-32, 4,0))
                .strafeTo(new Vector2d(-40,4));

        TrajectoryActionBuilder get3 = drive.actionBuilder(new Pose2d(-40,0,0))
                .strafeTo(new Vector2d(-59,-25));

        TrajectoryActionBuilder lineUp3 = drive.actionBuilder(new Pose2d(-45,-3,0))
                .strafeTo(new Vector2d(-32, 8)); // go forward to line up specimen
        // replace later with action of placing specimen
        TrajectoryActionBuilder reset3 = drive.actionBuilder(new Pose2d(-32, 8,0))
                .strafeTo(new Vector2d(-40,8));

        Action preload1Move = preLoad1.build();

        Action preload2Move = preload2.build();

        Action preload3Move = preLoad3.build();

        Action pushAction = pushSamples.build();

        Action unhook1 = unHook1.build();

        Action unhook2 = unHook2.build();

        Action moveToEnter = moveUp.build();

        Action lineupMove = lineUp.build();

        Action resetMove = reset.build();

        Action get1Move = get1.build();

        Action enterMove = enterPos.build();

        Action lineup1Move = lineUp1.build();

        Action reset1Move = reset1.build();

        Action get2Move = get2.build();

        Action lineup2Move = lineUp2.build();

        Action reset2Move = reset2.build();

        Action get3Move = get3.build();

        Action lineup3Move = lineUp3.build();

        Action reset3Move = reset3.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(

                        hzFourbarServo1.intakeMove(),
                        clawServo.openClaw(),
                        vFourbarServo1.fourBarReset(),
                        /*vFourbarServo1.fourBarGrab(),
                        clawServo.openClaw(),
                        vFourbarServo1.fourBarEnter()*/
                        preload1Move,
                        vFourbarServo1.fourBarEnter(),
                        preload2Move,
                        vFourbarServo1.fourBarPlace(),
                        preload3Move,
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarReset(),
                        pushAction,
                        vFourbarServo1.fourBarGrab(),
                        clawServo.openClaw(),
                        unhook1,
                        vFourbarServo2.fourBarUp(),
                        moveToEnter,
                        vFourbarServo1.fourBarEnter(),
                        lineupMove,
                        vFourbarServo1.fourBarPlace(),
                        resetMove,
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarReset(),
                        get1Move,
                        vFourbarServo1.fourBarGrab(),
                        clawServo.openClaw(),
                        unhook2,
                        vFourbarServo2.fourBarUp(),
                        enterMove,
                        vFourbarServo1.fourBarEnter(),
                        lineup1Move,
                        vFourbarServo1.fourBarPlace(),
                        reset1Move,
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarReset(),
                        get2Move,
                        vFourbarServo1.fourBarGrab(),
                        clawServo.openClaw(),
                        unhook2,
                        vFourbarServo2.fourBarUp(),
                        enterMove
                        /*
                        vFourbarServo1.fourBarEnter(),
                        lineup2Move,
                        vFourbarServo1.fourBarPlace(),
                        reset2Move,
                        clawServo.closeClaw(),
                        vFourbarServo1.fourBarReset()
                         */


                )
        );

    }

}
