package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Fight - short shovel", group="Xbotca")
public class fight01 extends LinearOpMode {

    public enum Commands {
        MOVE_LEFT_SIDE,
        MOVE_RIGHT_SIDE,
        LOW_SHOVEL_POS,
        CAPTURE_MINERALS,
        DROP_MINERALS,
        DRIVE_SHOVEL_POS,
        HIGH_SHOVEL_POS,
        ADJ_UP_SHOVEL,
        ADJ_DOWN_SHOVEL,
        HOOK_UP,
        HOOK_DOWN,
        VOID
    }

    private void LeftSideMoving () {
        mtFrontLeft.setPower(sidePowerR);
        mtFrontRight.setPower(sidePowerR);
        mtBackLeft.setPower(-sidePowerR*SIDE_MOVE_REAR_CORRECTION);
        mtBackRight.setPower(-sidePowerR*SIDE_MOVE_REAR_CORRECTION);
        telemetry.addData("Going Left", sidePowerR);
    }

    private void RightSideMoving () {
        mtFrontLeft.setPower(sidePowerR);
        mtFrontRight.setPower(sidePowerR);
        mtBackLeft.setPower(-sidePowerR*SIDE_MOVE_REAR_CORRECTION);
        mtBackRight.setPower(-sidePowerR*SIDE_MOVE_REAR_CORRECTION);
        telemetry.addData("Going Right", sidePowerR);
    }

    private void LowShovelPosition () {
        srvShovel.setPosition(0.62);
        grabber.setPower(0.15);
        mtArm.setPower(1);
        mtArm.setTargetPosition(-7500);
        telemetry.addData("Shovel position: ", mtArm.getCurrentPosition());
    }

    private void DriveShovelPosition () {
        grabber.setPower(-0.02); //0.15
        srvShovel.setPosition(0.3);
        grabber.setPower(-0.1);
        mtArm.setPower(1);
        mtArm.setTargetPosition(-1500);
        sleep(800);
        telemetry.addData("Shovel position: ", mtArm.getCurrentPosition());
    }

    private void HighShovelPosition () {
        grabber.setPower(-0.02); //0.15
        srvShovel.setPosition(0.3);
        mtArm.setPower(1);
        mtArm.setTargetPosition(16200);
        telemetry.addData("Shovel position: ", mtArm.getCurrentPosition());
    }

    private void CapturingMinerals () {
        grabber.setPower(0.1);
        srvShovel.setPosition(0.62);
        mtArm.setTargetPosition(-1500);
        while (mtArm.getCurrentPosition() < -3200) {
            sleep(100);
            telemetry.addData("Shovel position: ", mtArm.getCurrentPosition());
        }
        srvShovel.setPosition(0.3);
        //mtArm.setTargetPosition(-2000);

/*        srvShovel.setPosition(0.9);
        grabber.setPower(0.9);
        sleep(700);
        grabber.setPower(0);
        srvShovel.setPosition(0.5);
*/
    }

    private void DroppingMinerals () {
        /* RELEASE */
        grabber.setPower(-0.2);
        srvShovel.setPosition(0.90);
        sleep(1500);
        grabber.setPower(-0.06);
        srvShovel.setPosition(0.5);
    }

    private void ShovelGoingUP () {
        mtArm.setPower(0.75);
        telemetry.addData("Shovel is going up: ", mtArm.getCurrentPosition());
    }

    private void ShovelGoingDown () {
        mtArm.setPower(-0.75);
        telemetry.addData("Shovel is going down: ", mtArm.getCurrentPosition());
    }

    private void HookLowerPosition () {
        mtArm.setTargetPosition(0);
        hook1.setPower(-0.79);
        hook2.setPower(-0.79);  //going down 0.79
        telemetry.addData("Hook is going:", " DOWN");
    }

    private void HookUpperPosition () {
        hook1.setPower(0.08);
        hook2.setPower(0.07);
        telemetry.addData("Hook is going:", " UP");
    }

    private void DrivingMotorControl () {
        mtFrontLeft.setPower(tgtPowerL*SPEED_CORRECTION);
        mtFrontRight.setPower(tgtPowerR*SPEED_CORRECTION);
        mtBackLeft.setPower(tgtPowerL*SPEED_CORRECTION);
        mtBackRight.setPower(tgtPowerR*SPEED_CORRECTION);
        mtBackWheel.setPower(tgtPowerL);
        telemetry.addData("Left Power", tgtPowerL);
        telemetry.addData("Right Power", tgtPowerR);
    }

    private DcMotor mtFrontLeft, mtFrontRight, mtBackLeft, mtBackRight;
    private DcMotor mtArm, mtBackWheel;
    private CRServo hook1, hook2, grabber;
    private Servo srvShovel;


    Commands cmd;
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position
    static final double SIDE_MOVE_FRONT = 0.62;
    static final double SIDE_MOVE_REAR = 0.5;
    static final double SIDE_MOVE_REAR_CORRECTION = 0.9;
    static final double SPEED_CORRECTION = 0.8;
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    int lock_sensor0, lock_sensor1, lock_sensor2;
    double tgtPowerL = 0, tgtPowerR = 0, sidePowerL = 0, sidePowerR = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        final double STICK_ADJUST = 0.2;

        /*** Hardware Map ***/
        mtArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        srvShovel = hardwareMap.get(Servo.class, "shovel");
        grabber = hardwareMap.get(CRServo.class, "Grab");
        hook1 = hardwareMap.get(CRServo.class, "Hook1");
        hook2 = hardwareMap.get(CRServo.class, "Hook2");
        hook1.setDirection(DcMotorSimple.Direction.FORWARD);
        hook2.setDirection(DcMotorSimple.Direction.REVERSE);

        mtFrontRight = hardwareMap.get(DcMotor.class, "mt2");
        mtFrontLeft = hardwareMap.get(DcMotor.class, "mt3");
        mtBackRight = hardwareMap.get(DcMotor.class, "mt0");
        mtBackLeft = hardwareMap.get(DcMotor.class, "mt1");
        mtBackWheel = hardwareMap.get(DcMotor.class, "BackWheel");

        /*** ***/
        mtArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        mtArm.setPower(0);
//        mtArm.setTargetPosition(0);
//        grabber.setPower(0);
        srvShovel.setPosition(0.4);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shovel position: ", mtArm.getCurrentPosition());
        telemetry.update();

        waitForStart();

        /*** Main fighting cycle - minerals picking up and delivering ***/
        mtArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && !gamepad1.a) {
            tgtPowerL = -gamepad1.right_stick_y;
            tgtPowerR = gamepad1.left_stick_y;
            sidePowerR = gamepad1.right_stick_x;
            sidePowerL = gamepad1.left_stick_x;

            if (sidePowerL < -STICK_ADJUST && sidePowerR < -STICK_ADJUST)
                cmd = Commands.MOVE_LEFT_SIDE;
            else if (sidePowerL > STICK_ADJUST && sidePowerR > STICK_ADJUST)
                cmd = Commands.MOVE_RIGHT_SIDE;
            else if (sidePowerL > STICK_ADJUST && sidePowerR < -STICK_ADJUST)
                cmd = Commands.CAPTURE_MINERALS;
            else if (sidePowerL < -STICK_ADJUST && sidePowerR > STICK_ADJUST)
                cmd = Commands.DROP_MINERALS;
            else if (gamepad1.left_trigger > STICK_ADJUST)
                cmd = Commands.LOW_SHOVEL_POS;
            else if (gamepad1.right_bumper)
                cmd = Commands.HIGH_SHOVEL_POS;
            else if (gamepad1.left_bumper)
                cmd = Commands.DRIVE_SHOVEL_POS;
            else
                cmd = Commands.VOID;

            switch (cmd) {
                case MOVE_RIGHT_SIDE:
                    RightSideMoving();
                    break;
                case MOVE_LEFT_SIDE:
                    LeftSideMoving();
                    break;
                case LOW_SHOVEL_POS:
                    LowShovelPosition();
                    break;
                case HIGH_SHOVEL_POS:
                    HighShovelPosition();
                    break;
                case DRIVE_SHOVEL_POS:
                    DriveShovelPosition();
                    break;
                case CAPTURE_MINERALS:
                    CapturingMinerals();
                    break;
                case DROP_MINERALS:
                    DroppingMinerals();
                    break;
                case VOID:
                    DrivingMotorControl();
            }
//            telemetry.addData("Position: ", mtArm.getCurrentPosition());
            telemetry.update();
        }

        /*** Final stage - moving backwards, hook operating and shovel lift adjustment ***/
        mtArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            telemetry.addData("FINAL STAGE ", "- GET HOOKED NOW!!!");

            /*** Backward moving here ***/
            tgtPowerR = -gamepad1.right_stick_y;
            tgtPowerL = gamepad1.left_stick_y;
            sidePowerR = -gamepad1.right_stick_x;
            sidePowerL = -gamepad1.left_stick_x;

            if (sidePowerL < -STICK_ADJUST && sidePowerR < -STICK_ADJUST)
                cmd = Commands.MOVE_RIGHT_SIDE;
            else if (sidePowerL > STICK_ADJUST && sidePowerR > STICK_ADJUST)
                cmd = Commands.MOVE_LEFT_SIDE;
            else if (gamepad1.dpad_up)
                cmd = Commands.ADJ_UP_SHOVEL;
            else if (gamepad1.dpad_down)
                cmd = Commands.ADJ_DOWN_SHOVEL;
            else if (gamepad1.left_trigger > STICK_ADJUST)
                cmd = Commands.LOW_SHOVEL_POS;
            else if (gamepad1.right_bumper)
                cmd = Commands.HIGH_SHOVEL_POS;
            else if (gamepad1.left_bumper)
                cmd = Commands.DRIVE_SHOVEL_POS;
            else if (gamepad1.left_stick_button)
                cmd = Commands.HOOK_UP;
            else if (gamepad1.right_stick_button)
                cmd = Commands.HOOK_DOWN;
            else
                cmd = Commands.VOID;

            switch (cmd) {
                case MOVE_RIGHT_SIDE:
                    RightSideMoving();
                    break;
                case MOVE_LEFT_SIDE:
                    LeftSideMoving();
                    break;
                case LOW_SHOVEL_POS:
                    LowShovelPosition();
                    break;
                case HIGH_SHOVEL_POS:
                    HighShovelPosition();
                    break;
                case DRIVE_SHOVEL_POS:
                    DriveShovelPosition();
                    break;
                case HOOK_UP:
                    HookUpperPosition();
                    break;
                case HOOK_DOWN:
                    HookLowerPosition();
                    break;
                case ADJ_UP_SHOVEL:
                    ShovelGoingUP();
                    break;
                case ADJ_DOWN_SHOVEL:
                    ShovelGoingDown();
                    break;
                case VOID:
                    mtArm.setPower(0);
                    DrivingMotorControl();
            }
            telemetry.update();
        }
    }
}
