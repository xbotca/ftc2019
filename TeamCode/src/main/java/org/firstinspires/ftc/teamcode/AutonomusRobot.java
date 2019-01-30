package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by pdenisov on 12/21/2018.
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
public class AutonomusRobot extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftBack;
    private DcMotor RightBack;


    @Override
    public void runOpMode() throws InterruptedException {
        this.LeftFront = hardwareMap.get (DcMotor.class, "mt3");
        this.RightFront = hardwareMap.get (DcMotor.class, "mt2");
        this.LeftBack = hardwareMap.get (DcMotor.class, "mt1");
        this.RightBack = hardwareMap.get (DcMotor.class, "mt0");

        waitForStart();

        while (opModeIsActive()){
            LeftFront.setPower(1.0);
            RightFront.setPower(1.0);
            LeftBack.setPower(1.0);
            RightBack.setPower(1.0);

            sleep(1000);
            LeftFront.setPower(-1.0);
            RightFront.setPower(-1.0);
            LeftBack.setPower(-1.0);
            RightBack.setPower(-1.0);


        }
    }
}
