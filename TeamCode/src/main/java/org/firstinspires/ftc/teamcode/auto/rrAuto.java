package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

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
                clawServo.setPosition(0.5);
                sleep(1000);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }

    }

    public class intake {
        public Servo hzFourbarServo1;
        public Servo hzFourbarServo2;

        public intake(HardwareMap hardwareMap) {
            hzFourbarServo1 = hardwareMap.get(Servo.class, "hzSlidesServo1");
            hzFourbarServo2 = hardwareMap.get(Servo.class, "hzSlidesServo2");
        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ServoImplEx hzFourBarServo_1 = (ServoImplEx) hzFourbarServo1;
                ServoImplEx hzFourBarServo_2 = (ServoImplEx) hzFourbarServo2;
                hzFourBarServo_1.setPwmEnable();
                hzFourBarServo_2.setPwmEnable();
                return false;
            }
        }

        public Action intakeMove() {
            return new Intake();
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

        TrajectoryActionBuilder preLoad = drive.actionBuilder(beginPose)
                .setTangent(0)
                .strafeTo(new Vector2d(-41,0)); // move up to center rungs
                //slides up
        TrajectoryActionBuilder preLoad1 = drive.actionBuilder(new Pose2d(-41,0,0))
                .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen
                //slides down and unclip
                .waitSeconds(1.25); // replace later with action of placing specimen

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(-33,0,0))
                .strafeTo(new Vector2d(-46,0))

                .strafeTo(new Vector2d(-23,-22))
                .strafeTo(new Vector2d(-55,-25))

                .strafeTo(new Vector2d(-23,-22))
                .strafeTo(new Vector2d(-23,-28))
                .strafeTo(new Vector2d(-55,-28))

                .strafeTo(new Vector2d(-23,-28))
                .strafeTo(new Vector2d(-23,-34))
                .strafeTo(new Vector2d(-55,-34))


                .strafeTo(new Vector2d(-52,-11));

        TrajectoryActionBuilder get = drive.actionBuilder(new Pose2d(-33,0,0))
                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                .strafeTo(new Vector2d(-52,-33 ))

                .strafeTo(new Vector2d(-52,-11))

                .waitSeconds(0.5);

        TrajectoryActionBuilder place = drive.actionBuilder(new Pose2d(-52,-11,0))
                //.setTangent(Math.PI)
                .strafeTo(new Vector2d(-41,4));


        TrajectoryActionBuilder lineUp = drive.actionBuilder(new Pose2d(-41,4,0))
                .strafeTo(new Vector2d(-33, 4)) // go forward to line up specimen
                //slides down and unclip
                .waitSeconds(1.25); // replace later with action of placing specimen


        TrajectoryActionBuilder get1 = drive.actionBuilder(new Pose2d(-33,4,0))
                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                .strafeTo(new Vector2d(-52,-33 ))


                .strafeTo(new Vector2d(-52,-11))

                .waitSeconds(0.5);

        TrajectoryActionBuilder place1 = drive.actionBuilder(new Pose2d(-52,-11,0))
                //.setTangent(Math.PI)
                .strafeTo(new Vector2d(-41,8));

        TrajectoryActionBuilder lineUp1 = drive.actionBuilder(new Pose2d(-41,8,0))
                .strafeTo(new Vector2d(-33, 8)) // go forward to line up specimen
                //slides down and unclip
                .waitSeconds(1.25); // replace later with action of placing specimen

        TrajectoryActionBuilder get2 = drive.actionBuilder(new Pose2d(-33,8,0))
                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                .strafeTo(new Vector2d(-52,-33 ))



                .strafeTo(new Vector2d(-52,-11))

                .waitSeconds(0.5);

        TrajectoryActionBuilder place2 = drive.actionBuilder(new Pose2d(-52,-11,0))
                //.setTangent(Math.PI)
                .strafeTo(new Vector2d(-41,-4));


        TrajectoryActionBuilder lineUp2 = drive.actionBuilder(new Pose2d(-41,-4,0))
                .strafeTo(new Vector2d(-33, -4)) // go forward to line up specimen
                //slides down and unclip
                .waitSeconds(1.25); // replace later with action of placing specimen

        Action preloadMove = preLoad.build();

        Action preload1Move = preLoad1.build();

        Action getMove = get.build();

        Action placeMove = place.build();

        Action lineupMove = lineUp.build();

        Action get1Move = get1.build();

        Action place1Move = place1.build();

        Action lineup1Move = lineUp1.build();

        Action get2Move = get2.build();

        Action place2Move = place2.build();

        Action lineup2Move = lineUp2.build();

        Action pushAction = pushSamples.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(

                        hzFourbarServo1.intakeMove(),
                        preloadMove,
                        preload1Move,
                        pushAction,
                        getMove,
                        placeMove,
                        lineupMove,
                        get1Move,
                        place1Move,
                        lineup1Move



                        //.setTangent(0)
                        //.strafeToConstantHeading(new Vector2d(-60,-57))
                        /**clawServo.openClaw(),
                         outtakeMotor1.slidesUp(),
                         outtakeMotor1.slidesHold(),
                         clawServo.closeClaw(),
                         outtakeMotor1.slidesHold(),
                         outtakeMotor1.slidesDown()**/



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
