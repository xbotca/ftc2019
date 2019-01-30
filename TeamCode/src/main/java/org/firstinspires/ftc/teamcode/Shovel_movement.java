package org.firstinspires.ftc.teamcode;

/**
 * Created by pdenisov on 1/19/2019.
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
public class Shovel_movement extends LinearOpMode {
    private DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

            while(!gamepad1.a && opModeIsActive()) {
                if (gamepad1.x) {
                    armMotor.setPower(1);
                    telemetry.addData("X is pressed", gamepad1.x);

                }
                else if (gamepad1.y) {
                    armMotor.setPower(-1);
                    telemetry.addData("Y is pressed", gamepad1.y);

                }
                else {
                    armMotor.setPower(0);
                }
                telemetry.addData("Position: ", armMotor.getCurrentPosition());
                telemetry.update();
            }
    }
}
