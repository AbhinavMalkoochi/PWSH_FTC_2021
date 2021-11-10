package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class TeleFOC extends LinearOpMode {
    DcMotor spin;

    public void runOpMode() throws InterruptedException{
        spin = hardwareMap.get(DcMotor.class, "Spin");
        spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
            while (opModeIsActive()) {
                if(gamepad1.x){
                    spin.setPower(1);
                }
                else if(gamepad1.b){
                    spin.setPower(-1);
                }else{
                    spin.setPower(0);
                }


            }
    }
}
