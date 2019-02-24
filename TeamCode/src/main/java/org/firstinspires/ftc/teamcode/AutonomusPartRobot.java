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

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * Created by pdenisov on 12/21/2018.
 */
@Autonomous(name = "XBOT Crater Face", group="Xbotca")
public class AutonomusPartRobot extends LinearOpMode {
    /* Declare OpMode members. */
    private DcMotor mtFrontLeft, mtFrontRight, mtBackLeft, mtBackRight;
    private DcMotor mtArm, mtBackWheel;
    private CRServo hook1, hook2;
    private Servo srvShovel;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.4;

    static final double     SIDE_SPEED              = 0.3;
    static final double     SPEED_CORRECTION        = 0.9; //0.9
    static final double     SIDE_INCH_CORRECTION    = 1.2; //1.2

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AS6Pgk3/////AAABmVSmFw9rj0M0kA31aWnWzXV5N6kpx0LD5gFb19Nl801b4Z8O3glcKlT9a2HTFY8WiFmEnw0/rh4O0K/VBaiA61mxdDYpLauBFBNBWTqEteq2/JPtVS8J1x2UQGnBjyBmypfCAy6fOr/8uxKp9RrM8Q70W750ZtmkaAZsIaqNYPwqOjY1QDLms2tkqs+V88WTDd/KuPSQxXtPzTfUPDBMpFWbxaPUtcWrm2oDnk5qhQ6jX84GtVfMm5+61OGeZuzyctSKgT9w4/IEOSpF/fGPSFC9aor9Q0mnnmmwJKmgSCGLO/EqJCOt3MZD/z2gt/dx5sk3PFpbCD/5W03NTCPEO+ipDS9orNy4wBdYsaYGhjju";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

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

        mtArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        HookLowerPosition();
        srvShovel.setPosition(0.25);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d - %7d - %7d",
                mtFrontLeft.getCurrentPosition(),
                mtFrontRight.getCurrentPosition(),
                mtBackLeft.getCurrentPosition(),
                mtBackRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        mtArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        HookUpperPosition();
        mtArm.setTargetPosition(3600);
        mtArm.setPower(1);
        sleep(5500);


/*        / * SIDE DRIVING EXAMPLE - DON'T ERASE !! * /
        encoderDrive(DRIVE_SPEED,  11,  -11, 6.0);  // 90 DEGREE CLOCKWISE TURN
        sleep(2000);
        encoderSideDrive(SIDE_SPEED, -20, 3.0); // Negative means driving left
        sleep(2000);
        encoderSideDrive(SIDE_SPEED, 20, 3.0); // Positive is driving right
*/

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        encoderSideDrive(SIDE_SPEED, 4, 4.0);  // Side moving
        encoderDrive(DRIVE_SPEED,  4,  4, 5.0);
        encoderDrive(DRIVE_SPEED,  4,  -4, 5.0); // Small turn

       // HookLowerPosition();

        /* Just stop for control */
        sleep(1000);
////        encoderDrive(DRIVE_SPEED,  11,  -11, 6.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,  -11,  11, 6.0);  // 90 DEGREE COUNTERCLOCKWISE TURN
/*
        / * Lets start recognition * /
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (int i = 0; i < 4; i++) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 1) {
                        int goldMineralX = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1) {
                            telemetry.addData("Gold Mineral Position:", goldMineralX);
                            sleep(2000);  // temp
                            break;
                        }
                    } else if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineralX = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineralX == -1) {
                                silverMineralX = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineralX != -1) {
                            if (goldMineralX < silverMineralX) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                encoderSideDrive(SIDE_SPEED, 4, 4, 3.0);
                                break;
                            } else if (goldMineralX > silverMineralX) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                break;
                            }
                        }
                    } else {
                        encoderSideDrive(SIDE_SPEED, -4, -4, 3.0);
                    }
                    telemetry.update();
                }
            }
        }
*/
        /* Drive over*/
//        sleep(2000);
        encoderSideDrive(SIDE_SPEED, 10,3.0); // negative means left, positive is right
//        encoderDrive(DRIVE_SPEED,  10,  10, 6.0);  // S1: Forward 47 Inches with 5 Sec timeout

  //      sleep(600);
        encoderDrive(DRIVE_SPEED,  40,  40, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout

        encoderDrive(DRIVE_SPEED, -5.5, 5.5, 3.0);


        encoderSideDrive(DRIVE_SPEED, 10, 3.0);

        encoderDrive(DRIVE_SPEED, 50, 50, 3.0);
        sleep(600);

        srvShovel.setPosition(0.9);
        sleep(1000);

        srvShovel.setPosition(0.4);
//        sleep(600);

        encoderDrive(DRIVE_SPEED, -40, -40, 3.0);


        encoderSideDrive(DRIVE_SPEED, 10, 3.0);
        encoderSideDrive(DRIVE_SPEED, -10, 3.0);
        encoderDrive(DRIVE_SPEED, -20, 20, 3.0);  // -21.75
//        sleep(600);


        encoderDrive(1, 70, 70,3.0);
        sleep(300);

//        encoderDrive(1.3, 60, 60, 3.0);
        sleep(600);











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
        mtBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

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
            mtBackWheel.setTargetPosition(newBLeftTarget);

            // Turn On RUN_TO_POSITION
            mtFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            mtFrontLeft.setPower(Math.abs(speed));
            mtFrontRight.setPower(Math.abs(speed));
            mtBackLeft.setPower(Math.abs(speed));
            mtBackRight.setPower(Math.abs(speed));
            mtBackWheel.setPower(Math.abs(speed));


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
                        mtBackWheel.getCurrentPosition(),
                        mtBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            mtFrontLeft.setPower(0);
            mtFrontRight.setPower(0);
            mtBackLeft.setPower(0);
            mtBackRight.setPower(0);
            mtBackWheel.setPower(0);


            // Turn off RUN_TO_POSITION
            mtFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mtBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderSideDrive(double speed, double inches, double timeoutS) {
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
            newFLeftTarget = mtFrontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            newFRightTarget = mtFrontRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            newBLeftTarget = mtBackLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
            newBRightTarget = mtBackRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH * SIDE_INCH_CORRECTION);
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
            mtBackLeft.setPower(Math.abs(speed));
            mtBackRight.setPower(Math.abs(speed));
            mtFrontLeft.setPower(Math.abs(speed*SPEED_CORRECTION));
            mtFrontRight.setPower(Math.abs(speed*SPEED_CORRECTION));

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
    private void HookLowerPosition () {
        hook1.setPower(-0.79);
        hook2.setPower(-0.79);  //going down 0.79
        telemetry.addData("Hook is going:", " DOWN");
    }

    private void HookUpperPosition () {
        hook1.setPower(0.08);
        hook2.setPower(0.07);
        telemetry.addData("Hook is going:", " UP");
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
