package org.firstinspires.ftc.teamcode;

/**
 * Created by pdenisov on 10/30/2018.
 * Completed on / /2019.
 */

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.concurrent.TimeUnit;


@TeleOp
public class MechWheels_v1 extends LinearOpMode {

    private ColorSensor color_sensor1;
    private ColorSensor color_sensor2;
    private ColorSensor color_sensor0;

    private DcMotor mtFrontLeft, mtFrontRight, mtBackLeft, mtBackRight;
    private DcMotor armMotor, mtBack;

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    static final double SIDE_MOVE_FRONT = 0.62;
    static final double SIDE_MOVE_REAR = 0.5;

    // Define class members
    CRServo servoShovel;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    CRServo hook1;
    CRServo hook2;
    Servo lock0;
    Servo lock1;
    Servo lock2;
    Servo Door1;
    Servo Door2;
    Servo Door3;


    @Override
    public void runOpMode() throws InterruptedException {
        int lock_sensor0_blue, lock_sensor0_green, lock_sensor0_red;
        int lock_sensor1_blue, lock_sensor1_green, lock_sensor1_red;
        int lock_sensor2_blue, lock_sensor2_green, lock_sensor2_red;

        boolean pushFoundBall = true;
        boolean myTeamRed = true;
        int beforeRed, beforeBlue;
        int afterRed, afterBlue;
        int dBlue, dRed;
        double tgtPowerL = 0, tgtPowerR = 0;
        boolean tgtPowerSideL = false, tgtPowerSideR = false;
        int TopPosition = 20000;
        int BottomPosition = 50;
        int MiddlePosition = 9400;
        CRServo.Direction dir;
        servoShovel = hardwareMap.get(CRServo.class, "servoShovel");
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


        mtFrontRight = hardwareMap.get(DcMotor.class, "mt2");
//        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        mtFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        mtFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtFrontLeft = hardwareMap.get(DcMotor.class, "mt3");
//        mtFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mtFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtBackRight = hardwareMap.get(DcMotor.class, "mt0");
//        mtBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        mtBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mtBackLeft = hardwareMap.get(DcMotor.class, "mt1");
//        mtBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        mtBack = hardwareMap.get(DcMotor.class, "BackWheel");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*while(!gamepad1.a && opModeIsActive()){
            if(gamepad1.x) {
                armMotor.setPower(0.75);
                telemetry.addData("X is pressed", gamepad1.x);

            }
            else if(gamepad1.y){
                armMotor.setPower(-0.75);
                telemetry.addData("Y is pressed", gamepad1.y);

            }
            else{
                armMotor.setPower(0);
            }
            telemetry.addData("Position: ", armMotor.getCurrentPosition());
            telemetry.update();
        }*/

        waitForStart();
                /*while(!gamepad1.a && opModeIsActive()){
            if(gamepad1.x) {
                armMotor.setPower(0.75);
                telemetry.addData("X is pressed", gamepad1.x);

            }
            else if(gamepad1.y){
                armMotor.setPower(-0.75);
                telemetry.addData("Y is pressed", gamepad1.y);

            }
            else{
                armMotor.setPower(0);
            }
            telemetry.addData("Position: ", armMotor.getCurrentPosition());
            telemetry.update();
        }*/
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {

            lock_sensor0_blue = color_sensor0.blue();
            lock_sensor0_green = color_sensor0.green();
            lock_sensor0_red = color_sensor0.red();

            lock_sensor1_blue = color_sensor1.blue();
            lock_sensor1_green = color_sensor1.green();
            lock_sensor1_red = color_sensor1.red();

            lock_sensor2_blue = color_sensor2.blue();
            lock_sensor2_green = color_sensor2.green();
            lock_sensor2_red = color_sensor2.red();

            tgtPowerL = -gamepad1.right_stick_y;
            tgtPowerR = gamepad1.left_stick_y;
            tgtPowerSideL = gamepad1.dpad_left;
            tgtPowerSideR = gamepad1.dpad_right;


            if (gamepad1.dpad_right) {
                mtFrontLeft.setPower(SIDE_MOVE_FRONT);
                mtFrontRight.setPower(SIDE_MOVE_FRONT);
                mtBackLeft.setPower(-SIDE_MOVE_REAR);
                mtBackRight.setPower(-SIDE_MOVE_REAR);
                telemetry.addData("Going Right", tgtPowerSideR);

            } else if (gamepad1.dpad_left) {
                mtFrontLeft.setPower(-SIDE_MOVE_FRONT);
                mtFrontRight.setPower(-SIDE_MOVE_FRONT);
                mtBackLeft.setPower(SIDE_MOVE_REAR);
                mtBackRight.setPower(SIDE_MOVE_REAR);
                telemetry.addData("Going Left", tgtPowerSideL);
            } else {
                mtFrontLeft.setPower(tgtPowerL);
                mtFrontRight.setPower(tgtPowerR);
                mtBackLeft.setPower(tgtPowerL);
                mtBackRight.setPower(tgtPowerR);
                mtBack.setPower(tgtPowerL);
                telemetry.addData("Left Power", tgtPowerL);
                telemetry.addData("Right Power", tgtPowerR);
            }
            if (gamepad1.y) {
                armMotor.setPower(1);
                armMotor.setTargetPosition(TopPosition);
                telemetry.addData("Y is pressed", gamepad1.y);
            } else if (gamepad1.x) {
                armMotor.setPower(1);
                armMotor.setTargetPosition(0);
                telemetry.addData("X is pressed", gamepad1.x);
            } else if (gamepad1.b) {
                armMotor.setPower(1);
                armMotor.setTargetPosition(MiddlePosition);
                telemetry.addData("B is pressed", gamepad1.b);
            }


            if (gamepad1.right_bumper) {
                //hook1.setPower(-0.79);
                //hook2.setPower(0.79);  //going down 0.79
                hook1.setPower(-0.79);
                hook2.setPower(-0.79);  //going down 0.79
            } else if (gamepad1.left_bumper) {
                //hook1.setPower(1.1);
                //hook2.setPower(-0.7);  //going up -0.4
                hook1.setPower(0);
                hook2.setPower(0);  //going up -0.4
            } else if (gamepad1.a) {
                hook1.setPower(0.08);
                hook2.setPower(0.07);
            }


//            if(lock_sensor0_red > 1400 && lock_sensor0_green > 1200) {
//                // An yellow cube is here
//                lock0.setPosition(0);
//                sleep(300);
//            }
//            if(lock_sensor0_red > 2000 && lock_sensor0_blue > 2000 && lock_sensor0_green > 2000) {
//                // A white ball is here
//                lock0.setPosition(0);
//                sleep(300);
//            }
//
//            if(lock_sensor1_red > 1400 && lock_sensor1_green > 1200) {
//                // An yellow cube is here
//                lock1.setPosition(0);
//                sleep(300);
//            }
//            if(lock_sensor1_red > 2000 && lock_sensor1_blue > 2000 && lock_sensor1_green > 2000) {
//                // A white ball is here
//                lock1.setPosition(0);
//                sleep(300);
//            }
//
//            if(lock_sensor2_red > 1400 && lock_sensor2_green > 1200) {
//                // An yellow cube is here
//                lock2.setPosition(0.4);
//                sleep(300);
//
//            }
//            if(lock_sensor2_red > 2000 && lock_sensor2_blue > 2000 && lock_sensor2_green > 2000) {
//                // A white ball is here
//                lock2.setPosition(0.7);
//                sleep(300);
//            }
            String colorOf0 = determineColor(color_sensor0);
            String colorOf1 = determineColor(color_sensor1);
            String colorOf2 = determineColor(color_sensor2);

            if (colorOf0 == "yellow"){
                lock0.setPosition(-1);
            }
            if (colorOf0 == "yellow"){
                lock1.setPosition(-1);
            }
            if (colorOf0 == "yellow"){
                lock2.setPosition(-1);
            }
           telemetry.addData("color of 0  : ", colorOf0);
           telemetry.addData("color of 1  : ", colorOf1);
            telemetry.addData("color of 2 : ", colorOf2);


            telemetry.addData("Redness of clr0 : ",lock_sensor0_red );
            telemetry.addData("Blueness of clr0 : ", lock_sensor0_blue);
            telemetry.addData("Greenness of clr0 : ", lock_sensor0_green);
           telemetry.addData("Redness of clr1 : ", lock_sensor1_red);
           telemetry.addData("Blueness of clr1 : ", lock_sensor1_blue);
           telemetry.addData("Greenness of clr1 : ", lock_sensor1_green);
//
            telemetry.addData("Redness of clr2 : ", lock_sensor2_red);
            telemetry.addData("Blueness of clr2 : ", lock_sensor2_blue);
            telemetry.addData("Greenness of clr2 : ", lock_sensor2_green);
            telemetry.addData("Position: ", armMotor.getCurrentPosition());

            telemetry.update();
        }

    }


    public String determineColor(ColorSensor colorSensor) {
        if (colorSensor.red() >= 1300 && colorSensor.green() >= 1300 && colorSensor.blue() >= 1400) {
            return "white";
        } else if (colorSensor.red() >= 1300 && colorSensor.green() >= 1300) {
            return "yellow";
        }
        return "nothing";
    }
}
