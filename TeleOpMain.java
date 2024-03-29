package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp

public class TeleOpMain extends LinearOpMode {

    DcMotor motorFL, motorBL, motorBR, motorFR, motorLinearSlide, intakeHex, motorFLSecond, duckMotor;
    Servo rotateClawServo;//, clawServo;
    public void runOpMode() throws InterruptedException{

        //Mapping Motors
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        motorBL = hardwareMap.get(DcMotor.class,"BackLeftMotor");
        motorBR = hardwareMap.get(DcMotor.class,"BackRightMotor");
        motorFR = hardwareMap.get(DcMotor.class,"FrontRightMotor");
        duckMotor = hardwareMap.get(DcMotor.class,"DuckMotor");
        motorFLSecond = hardwareMap.get(DcMotor.class,"FrontLeftMotorSecond");
        intakeHex = hardwareMap.get(DcMotor.class, "IntakeHexMotor");
        motorLinearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        rotateClawServo = hardwareMap.get(Servo.class,"RotateServo");
        //clawServo = hardwareMap.get(Servo.class,"ClawServo");
        intakeHex.setPower(0.0);
        motorLinearSlide.setPower(0.0);
        motorFLSecond.setPower(0.0);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

  /*      motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
*/
        waitForStart();
        while (opModeIsActive()) {

            //driver gamepad control

            double y = -gamepad1.right_stick_x*0.8;
            double x = gamepad1.left_stick_x*0.8;
            double rx = gamepad1.left_stick_y*0.8;

            /*
            double intakeHexPower = gamepad1.a*0.3;
            double linearSlidePower = gamepad1.b*0.3;
            double clawPower = gamepad1.y*0.3;
            double rotateClawPower = gamepad1.x*0.3;
            */

            double motorFLPower = ((y + x + rx));
            double motorBLPower = ((y - x + rx));
            double motorFRPower = ((y - x - rx));
            double motorBRPower = ((y + x - rx));
            double motorFLSecondPower = ((y + x + rx));
            motorFL.setPower(motorFLPower);
            motorFLSecond.setPower(motorFLSecondPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);



            if(gamepad1.y){
                rotateClawServo.setPosition(rotateClawServo.getPosition()+0.001);
            }
            if(gamepad1.a){
                rotateClawServo.setPosition(rotateClawServo.getPosition()-0.001);
            }

            if(gamepad1.b){
                motorLinearSlide.setPower(1);
            }else if(gamepad1.x){
                motorLinearSlide.setPower(-1);
            }else{
                motorLinearSlide.setPower(0);
            }

            if(gamepad1.dpad_up){
                intakeHex.setPower(1);
            }
            else if(gamepad1.dpad_down){
                intakeHex.setPower(-1);
            }else{
                intakeHex.setPower(0);
            }

            telemetry.addData("COED EOWKR  ", "pp");


            telemetry.update();


            /*
            if(gamepad1.x){
                rotateClawServo.setPosition(rotateClawServo.getPosition()+0.005);
            }
            */

        }
    }
}
