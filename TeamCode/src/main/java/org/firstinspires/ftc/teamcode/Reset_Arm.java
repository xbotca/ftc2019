package org.firstinspires.ftc.teamcode;

/**
 * Created by pdenisov on 10/30/2018.
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

@TeleOp
public class Reset_Arm extends LinearOpMode {

    private ColorSensor color_sensor1;
    private Servo hand;
    private DcMotor mtFrontLeft, mtFrontRight, mtBackLeft, mtBackRight;
    private DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean pushFoundBall = true;
        boolean myTeamRed = true;
        int beforeRed, beforeBlue;
        int afterRed, afterBlue;
        int dBlue, dRed;
        double tgtPowerL = 0, tgtPowerR = 0;
        boolean tgtPowerSideL = false, tgtPowerSideR = false;
        int TopPosition = 19000;
        int BottomPosition = 100;
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



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        while(!gamepad1.a && opModeIsActive()){
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
        }
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()){
            tgtPowerL = gamepad1.left_stick_y;
            tgtPowerR = -gamepad1.right_stick_y;
            tgtPowerSideL = gamepad1.dpad_left;
            tgtPowerSideR = gamepad1.dpad_right;


            if(gamepad1.dpad_right){
                mtFrontLeft.setPower(-0.5);
                mtFrontRight.setPower(0.5);
                mtBackLeft.setPower(0.5);
                mtBackRight.setPower(-0.5);
                telemetry.addData("Going Right", tgtPowerSideR);

            }

            else if(gamepad1.dpad_left){
                mtFrontLeft.setPower(0.5);
                mtFrontRight.setPower(-0.5);
                mtBackLeft.setPower(-0.5);
                mtBackRight.setPower(0.5);
                telemetry.addData("Going Left", tgtPowerSideL);
            }
            else {
                mtFrontLeft.setPower(tgtPowerL);
                mtFrontRight.setPower(-tgtPowerR);
                mtBackLeft.setPower(tgtPowerL);
                mtBackRight.setPower(-tgtPowerR);
                telemetry.addData("Left Power", tgtPowerL);
                telemetry.addData("Right Power", tgtPowerR);}
          /* if(gamepad1.y) {
                while(armMotor.getTargetPosition() > 1800) {
                    armMotor.setPower(0.75);
                    armMotor.setTargetPosition(TopPosition);
                    telemetry.addData("Y is pressed", gamepad1.y);
                }

            }
            else if(gamepad1.x){
               while(armMotor.getTargetPosition() < 150) {
                   armMotor.setPower(0.75);
                   armMotor.setTargetPosition(BottomPosition);
                   telemetry.addData("X is pressed", gamepad1.x);
               }

            }
            else{
                armMotor.setPower(0);
            }*/
            telemetry.addData("Position: ", armMotor.getCurrentPosition());

            telemetry.update();
        }

    }

}
