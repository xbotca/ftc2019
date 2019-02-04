package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by pdenisov on 12/21/2018.
 */
@Autonomous(name = "Concept: XBOT Auto Drive By Encoder", group="Xbotca")
public class AutonomusRobot extends LinearOpMode {
    /* Declare OpMode members. */
    private ColorSensor color_sensor0, color_sensor1, color_sensor2;
    private DcMotor mtFrontLeft, mtFrontRight, mtBackLeft, mtBackRight;
    private DcMotor mtArm, mtBackWheel;
    private CRServo hook1, hook2;
    private Servo lock0, lock1, lock2, srvShovel;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.4;

    static final double     SIDE_SPEED              = 0.3;
    static final double     SPEED_CORRECTION        = 0.9;
    static final double     SIDE_INCH_CORRECTION    = 1.25;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        /*** Hardware Map ***/
        mtArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        srvShovel = hardwareMap.get(Servo.class, "shovel");
        hook1 = hardwareMap.get(CRServo.class, "Hook1");
        hook2 = hardwareMap.get(CRServo.class, "Hook2");
        hook1.setDirection(DcMotorSimple.Direction.FORWARD);
        hook2.setDirection(DcMotorSimple.Direction.REVERSE);

        color_sensor0 = hardwareMap.get(ColorSensor.class, "clr0");
        color_sensor1 = hardwareMap.get(ColorSensor.class, "clr1");
        color_sensor2 = hardwareMap.get(ColorSensor.class, "clr2");

        lock0 = hardwareMap.get(Servo.class, "lock0");
        lock1 = hardwareMap.get(Servo.class, "lock1");
        lock2 = hardwareMap.get(Servo.class, "lock2");

        mtFrontRight = hardwareMap.get(DcMotor.class, "mt3");
        mtFrontLeft = hardwareMap.get(DcMotor.class, "mt2");
        mtBackRight = hardwareMap.get(DcMotor.class, "mt1");
        mtBackLeft = hardwareMap.get(DcMotor.class, "mt0");
        mtBackWheel = hardwareMap.get(DcMotor.class, "BackWheel");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        mtFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d - %7d - %7d",
                mtFrontLeft.getCurrentPosition(),
                mtFrontRight.getCurrentPosition(),
                mtBackLeft.getCurrentPosition(),
                mtBackRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  50,  50, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout

        encoderSideDrive(SIDE_SPEED, -40, -40, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(5000);
        encoderSideDrive(SIDE_SPEED, 40, 40, 10.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//        encoderDrive(TURN_SPEED,   22, -22, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED,  45,  45, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

//        encoderSideDrive(DRIVE_SPEED, 20, 20, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFLeftTarget;
        int newFRightTarget;
        int newBLeftTarget;
        int newBRightTarget;

        mtFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        mtBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        mtFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mtBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLeftTarget = mtFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFRightTarget = mtFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBLeftTarget = mtBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRightTarget = mtBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            mtFrontLeft.setTargetPosition(newFLeftTarget);
            mtFrontRight.setTargetPosition(newFRightTarget);
            mtBackLeft.setTargetPosition(newBLeftTarget);
            mtBackRight.setTargetPosition(newBRightTarget);

            // Turn On RUN_TO_POSITION
            mtFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            mtFrontLeft.setPower(Math.abs(speed));
            mtFrontRight.setPower(Math.abs(speed));
            mtBackLeft.setPower(Math.abs(speed));
            mtBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mtFrontLeft.isBusy() && mtFrontRight.isBusy() && mtBackLeft.isBusy() && mtBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d - %7d - %7d", newFLeftTarget,
                        newFRightTarget, newBLeftTarget,  newBRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d - %7d - %7d",
                        mtFrontLeft.getCurrentPosition(),
                        mtFrontRight.getCurrentPosition(),
                        mtBackLeft.getCurrentPosition(),
                        mtBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mtFrontLeft.setPower(0);
            mtFrontRight.setPower(0);
            mtBackLeft.setPower(0);
            mtBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            mtFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderSideDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFLeftTarget;
        int newFRightTarget;
        int newBLeftTarget;
        int newBRightTarget;

        mtFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        mtBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mtFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mtBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLeftTarget = mtFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            newFRightTarget = mtFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            newBLeftTarget = mtBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            newBRightTarget = mtBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            mtFrontLeft.setTargetPosition(newFLeftTarget);
            mtFrontRight.setTargetPosition(newFRightTarget);
            mtBackLeft.setTargetPosition(newBLeftTarget);
            mtBackRight.setTargetPosition(newBRightTarget);

            // Turn On RUN_TO_POSITION
            mtFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            mtFrontLeft.setPower(Math.abs(speed*SPEED_CORRECTION));
            mtFrontRight.setPower(Math.abs(speed*SPEED_CORRECTION));
            mtBackLeft.setPower(Math.abs(speed));
            mtBackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mtFrontLeft.isBusy() && mtFrontRight.isBusy() && mtBackLeft.isBusy() && mtBackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d - %7d - %7d", newFLeftTarget,
                        newFRightTarget, newBLeftTarget,  newBRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d - %7d - %7d",
                        mtFrontLeft.getCurrentPosition(),
                        mtFrontRight.getCurrentPosition(),
                        mtBackLeft.getCurrentPosition(),
                        mtBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mtFrontLeft.setPower(0);
            mtFrontRight.setPower(0);
            mtBackLeft.setPower(0);
            mtBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            mtFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
