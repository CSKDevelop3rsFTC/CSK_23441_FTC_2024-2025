package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "odoTest")
public class odoTest extends LinearOpMode {

    odoPinPointDriver odo;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    //4 Stage Slides Motors (Outtake)
    public DcMotor outtakeMotor1 = null;
    public DcMotor outtakeMotor2 = null;

    //Intake Servos
    public Servo hzfourbarServo1 = null;
    public Servo hzfourbarServo2 = null;
    public Servo hzSlidesServo1 = null;
    public Servo hzSlidesServo2 = null;
    public CRServo intakeServo = null;

    //Outtake Servos
    public Servo vFourbarServo1 = null;
    public Servo vFourbarServo2 = null;
    public Servo clawRotateServo = null;
    public Servo clawServo = null;

    public double driveSpeed = 0.65;
    public double intakeSpeed = 0.3;


    public void stopRobot() {
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
    }

    public void vertMove(double dir, int time) { //inputs for dir is -1 or 1 (forward or backward)
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

    public void horizMove(double dir, int time) { // -1 for left 1 for positive  for
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

    public void pivot(double dir, int time) { // -1 for turnLeft 1 for turnRight
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

    public void spinTake(double dir, int time) {
        intakeServo.setPower(intakeSpeed * dir);
        sleep(time);
    }


    public void runOpMode() {
        odo = hardwareMap.get(odoPinPointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(odoPinPointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(odoPinPointDriver.EncoderDirection.FORWARD, odoPinPointDriver.EncoderDirection.FORWARD);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        outtakeMotor1 = hardwareMap.get(DcMotor.class, "outtakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotor.class, "outtakeMotor2");

        hzfourbarServo1 = hardwareMap.get(Servo.class, "hzfourbarServo1");
        hzfourbarServo2 = hardwareMap.get(Servo.class, "hzfourbarServo2");
        hzSlidesServo1 = hardwareMap.get(Servo.class, "hzSlidesServo1");
        hzSlidesServo2 = hardwareMap.get(Servo.class, "hzSlidesServo2");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        vFourbarServo1 = hardwareMap.get(Servo.class, "vFourbarServo1");
        vFourbarServo2 = hardwareMap.get(Servo.class, "vFourbarServo2");
        clawRotateServo = hardwareMap.get(Servo.class, "clawRotateServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        odo.resetPosAndIMU();
        telemetry.addData("Status", "Ready to Start");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            odo.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;
*/

            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            //telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }

        stopRobot();
    }
}