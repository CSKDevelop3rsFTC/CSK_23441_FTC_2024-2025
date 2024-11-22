package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.teleop.Meet0TeleOp;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "autoTest", group = "Autonomous")
public class autoTest extends LinearOpMode {




    public class Intake{
        public class IntakeSample implements Action {
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    auto.hzFourBar(1);
                    auto.spinTake(1, 300);
                    initialized = true;
                }

                double pos1 = auto.hzFourbarServo1.getPosition();
                double pos2 = auto.hzFourbarServo2.getPosition();
                if (pos1 == 0 && pos2 == 0) {
                    return true;
                } else {
                    auto.intakeServo.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeSample() {
            return new IntakeSample();
        }

        public class IntakeBack implements Action {
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    auto.hzFourBar(0);
                    auto.intakeServo.setPower(0);
                    initialized = true;
                }

                double pos1 = auto.hzFourbarServo1.getPosition();
                double pos2 = auto.hzFourbarServo2.getPosition();
                if (pos1 == 0.95 && pos2 == 0.95) {
                    return true;
                } else {
                    auto.spinTake(-1, 200);
                    return false;
                }
            }
        }

        public Action intakeBack() {
            return new IntakeBack();
        }
    }
    public class Outtake {
        public class LiftSlides implements Action{
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    auto.move(auto.outtakeMotor1.getCurrentPosition()+300, DcMotorSimple.Direction.REVERSE,1);
                    auto.move2(auto.outtakeMotor2.getCurrentPosition() + 300, DcMotorSimple.Direction.FORWARD, 1);
                    initialized = true;
                }
                double pos1 = auto.outtakeMotor1.getCurrentPosition();
                double pos2 = auto.outtakeMotor2.getCurrentPosition();
                if(pos1 < 6000 && pos2 < 6000){
                    return true;
                } else {
                    auto.outtakeMotor1.setPower(0);
                    auto.outtakeMotor2.setPower(0);
                    return false;
                }


            }
        }
        public Action liftSlides(){
            return new LiftSlides();
        }
        public class ResetSlides implements Action{
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    auto.move(50, DcMotorSimple.Direction.REVERSE, 0.95);
                    auto.move2(50, DcMotorSimple.Direction.FORWARD,0.95);
                    initialized = true;
                }
                double pos1 = auto.outtakeMotor1.getCurrentPosition();
                double pos2 = auto.outtakeMotor2.getCurrentPosition();
                if(pos1 > 60 && pos2 > 60){
                    return true;
                } else {
                    auto.outtakeMotor1.setPower(0);
                    auto.outtakeMotor2.setPower(0);
                    return false;
                }


            }
        }
        public Action resetSlides(){
            return new ResetSlides();
        }

        public class LiftFourBar implements Action{
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    auto.slidesFourBar(1);
                    initialized = true;
                }
                double pos1 = auto.vFourbarServo1.getPosition();
                double pos2 = auto.vFourbarServo2.getPosition();
                if(pos1 == 0 && pos2 == 0){
                    return true;
                } else {
                    return false;
                }

            }
        }
        public Action liftFourBar(){
            return new LiftFourBar();
        }
        public class ResetFourBar implements Action{
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    auto.slidesFourBar(0);
                    initialized = true;
                }
                double pos1 = auto.vFourbarServo1.getPosition();
                double pos2 = auto.vFourbarServo2.getPosition();
                if(pos1 == 0.4 && pos2 == 1 || pos1 == 0.25 && pos2 == 0.5){
                    return true;
                } else {
                    return  false;
                }

            }
        }
        public Action resetFourBar(){
            return new ResetFourBar();
        }
        public class HoldFourBar implements Action{
            autoBasic auto = new autoBasic();
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    auto.slidesFourBar(2);
                    initialized = true;
                }
                double pos1 = auto.vFourbarServo1.getPosition();
                double pos2 = auto.vFourbarServo2.getPosition();
                if(pos1 == 0.4 && pos2 == 1){
                    return true;
                } else {
                    return  false;
                }

            }
        }
        public Action holdFourBar(){
            return new HoldFourBar();
        }

    }

    public class Claw {
        public class OpenClaw implements Action {
            autoBasic auto = new autoBasic();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                auto.clawPosition(1);
                return false;
            }
        }
        public Action openClaw(){
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            autoBasic auto = new autoBasic();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                auto.clawPosition(0);
                return false;
            }
        }
        public Action closeClaw(){
            return new CloseClaw();
        }
    }



    public void runOpMode() {
        Intake intake = new Intake();
        Claw claw = new Claw();
        Outtake outtake = new Outtake();
        autoBasic auto = new autoBasic();

        // instantiate your MecanumDrive at a particular pose.
        Pose2d beginPose = new Pose2d(-63, -11, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2) // move up to center rungs
                .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-33, 0, Math.PI/2))
                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit
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
                .splineToConstantHeading(new Vector2d(0,-24), Math.PI / 2); // move up to center rungs
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit
                .build();
        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        Action trajectoryChosen2 = tab2.build();
        Action trajectoryChosen3 = tab3.build();
        Action trajectoryChosen4 = tab4.build();
        Action trajectoryChosen5 = tab5.build();
        Action trajectoryChosen6 = tab6.build();

        Actions.runBlocking(
                new SequentialAction(
                        //trajectoryActionChosen,
                        //trajectoryChosen2,
                        intake.intakeSample(),
                        intake.intakeBack()
                        /**trajectoryChosen3,
                        claw.openClaw(),
                        outtake.liftFourBar(),
                        claw.closeClaw(),
                        outtake.holdFourBar(),
                        trajectoryChosen4,
                        intake.intakeSample(),
                        intake.intakeBack(),
                        trajectoryChosen5,
                        outtake.resetFourBar(),
                        claw.openClaw(),
                        outtake.liftFourBar(),
                        claw.closeClaw(),
                        outtake.holdFourBar(),
                        trajectoryChosen6**/

                )
        );


    }

}