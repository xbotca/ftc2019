package org.firstinspires.ftc.teamcode;

/**
 * Created by pdenisov on 10/30/2018.
 * Completed on / /2019.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Shovel_v1 extends LinearOpMode {

    private ColorSensor color_sensor1;
    private ColorSensor color_sensor2;
    private ColorSensor color_sensor0;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    CRServo servoShovel;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    CRServo hook1;
    CRServo hook2;
    Servo lock0;
    Servo lock1;
    Servo lock2;

    Servo handServo;

    boolean lock0yellow, lock0white;
    boolean lock1yellow, lock1white;
    boolean lock2yellow, lock2white;


    DcMotor backWheel;
    @Override
    public void runOpMode() throws InterruptedException {
        int lock_sensor0_blue, lock_sensor0_green, lock_sensor0_red;
        int lock_sensor1_blue, lock_sensor1_green, lock_sensor1_red;
        int lock_sensor2_blue, lock_sensor2_green, lock_sensor2_red;

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

        color_sensor0 = hardwareMap.get(ColorSensor.class, "clr1");
        color_sensor1 = hardwareMap.get(ColorSensor.class, "clr0");
        color_sensor2 = hardwareMap.get(ColorSensor.class, "clr2");

        lock0 = hardwareMap.get(Servo.class, "lock1");
        lock1 = hardwareMap.get(Servo.class, "lock0");
        lock2 = hardwareMap.get(Servo.class, "lock2");

        handServo = hardwareMap.get(Servo.class, "handservo");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

//            handServo.setPosition(0.5);

/*            lock0.setPosition(0.5);
            lock1.setPosition(0.5);
            lock2.setPosition(0.5);
*/
            tgtPowerL = -gamepad1.right_stick_y;
            tgtPowerR = gamepad1.left_stick_y;
            tgtPowerSideL = gamepad1.dpad_left;
            tgtPowerSideR = gamepad1.dpad_right;

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

            lock0white = false;
            lock1white = false;
            lock2white = false;
            lock0yellow = false;
            lock1yellow = false;
            lock2yellow = false;

            if(lock_sensor0_red > 1400 && lock_sensor0_green > 1200 && lock_sensor0_blue < 1500) {
                // An yellow cube is here
                lock0.setPosition(0.4);
                lock0yellow = true;
                sleep(300);
            }
            else if(lock_sensor0_red > 2000 && lock_sensor0_blue > 2000 && lock_sensor0_green > 2000) {
                // A white ball is here
                lock0.setPosition(0.7);
                lock0white = true;
                sleep(300);
            }

            if(lock_sensor1_red > 1400 && lock_sensor1_green > 1200 && lock_sensor1_blue < 1500) {
                // An yellow cube is here
                lock1.setPosition(0.4);
                lock1yellow = true;
                sleep(300);
            }
            else if(lock_sensor1_red > 2000 && lock_sensor1_blue > 2000 && lock_sensor1_green > 2000) {
                // A white ball is here
                lock1.setPosition(0.7);
                lock1white = true;
                sleep(300);
            }

            if(lock_sensor2_red > 1400 && lock_sensor2_green > 1200 && lock_sensor2_blue < 1500) {
                // An yellow cube is here
                lock2.setPosition(0.4);
                lock2yellow = true;
                sleep(300);

            }
            else if(lock_sensor2_red > 2000 && lock_sensor2_blue > 2000 && lock_sensor2_green > 2000) {
                // A white ball is here
                lock2.setPosition(0.7);
                lock2white = true;
                sleep(300);
            }


            if(gamepad1.dpad_right){
                telemetry.addData("Going Right", tgtPowerSideR);

            }

            else if(gamepad1.dpad_left){
                telemetry.addData("Going Left", tgtPowerSideL);
            }
            else {
                telemetry.addData("Left Power", tgtPowerL);
                telemetry.addData("Right Power", tgtPowerR);
            }
          if(gamepad1.y) {
              telemetry.addData("Y is pressed", gamepad1.y);
            }
            else if(gamepad1.x){
              telemetry.addData("X is pressed", gamepad1.x);
            }
          else if(gamepad1.b){
              telemetry.addData("B is pressed", gamepad1.b);
          }

          if (lock0yellow && lock1yellow && lock2yellow){
              lock0.setPosition(1);
              sleep(400);
          }
          if (lock0white && lock1white && lock2white){
                lock0.setPosition(1);
                sleep(400);
          }
/*
          if(gamepad1.right_bumper){
              //hook1.setPower(-0.79);
              //hook2.setPower(0.79);  //going down 0.79
              hook1.setPower(-0.79);
              hook2.setPower(-0.79);  //going down 0.79
          }
          else if(gamepad1.left_bumper){
              //hook1.setPower(1.1);
              //hook2.setPower(-0.7);  //going up -0.4
              hook1.setPower(0);
              hook2.setPower(0);  //going up -0.4
          }
          else if(gamepad1.a){
              hook1.setPower(0.08);
              hook2.setPower(0.07);
          }
*/
            if (gamepad1.x) {
              //let go all of the whites
              if (lock0white){
                  lock0.setPosition(1);
              }
              if (lock1white){
                  lock1.setPosition(1);
              }
              if (lock2white){
                  lock2.setPosition(1);
              }

          }
          if (gamepad1.y) {
                //let go all of the yellows
              if (lock0yellow){
                  lock0.setPosition(1);
              }
              if (lock1yellow){
                  lock1.setPosition(1);
              }
              if (lock2yellow){
                  lock2.setPosition(1);

              }

          }

            telemetry.addLine("Color sensor 0: ")
                    .addData("red: ", lock_sensor0_red)
                    .addData("green: ", lock_sensor0_green)
                    .addData("blue: ", lock_sensor0_blue);

            telemetry.addLine("Color sensor 1: ")
                    .addData("red: ", lock_sensor1_red)
                    .addData("green: ", lock_sensor1_green)
                    .addData("blue: ", lock_sensor1_blue);

            telemetry.addLine("Color sensor 2: ")
                    .addData("red: ", lock_sensor2_red)
                    .addData("green: ", lock_sensor2_green)
                    .addData("blue: ", lock_sensor2_blue);

            telemetry.update();
//            lock0.setPosition(1.0);
            sleep(100);

        }

    }

}
