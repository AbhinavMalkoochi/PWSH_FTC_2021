package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp

public class TeleOpMain extends LinearOpMode {

    DcMotor motorFL, motorBL, motorBR, motorFR,intakeHex, motorLinearSlide;
    Servo rotateClawServo, clawServo;
    public void runOpMode() throws InterruptedException{

        //Mapping Motors
        motorFL = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        motorBL = hardwareMap.get(DcMotor.class,"BackLeftMotor");
        motorBR = hardwareMap.get(DcMotor.class,"BackRightMotor");
        motorFR = hardwareMap.get(DcMotor.class,"FrontRightMotor");
        intakeHex = hardwareMap.get(DcMotor.class, "IntakeHexMotor");
        motorLinearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        rotateClawServo = hardwareMap.get(Servo.class,"RotateServo");
        clawServo = hardwareMap.get(Servo.class,"ClawServo");
        intakeHex.setPower(0.0);

        waitForStart();

        while (opModeIsActive()) {

            //driver gamepad control
            double y = -gamepad1.left_stick_y*0.7;
            double x = gamepad1.left_stick_x*0.7;
            double dx = gamepad1.right_stick_x*0.7;
            /*
            double intakeHexPower = gamepad1.a*0.3;
            double linearSlidePower = gamepad1.b*0.3;
            double clawPower = gamepad1.y*0.3;
            double rotateClawPower = gamepad1.x*0.3;
            */
            double motorFLPower = (y + x + dx);
            double motorBLPower = (y - x + dx);
            double motorFRPower = (y - x - dx);
            double motorBRPower = (y + x - dx);

            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);

            if(gamepad1.a){
                intakeHex.setPower(0.3);
            }
            if(gamepad1.b){
                motorLinearSlide.setPower(0.3);
            }
            if(gamepad1.y){
                clawServo.setPosition(1.0);
            }
            if(gamepad1.x){
                rotateClawServo.setPosition(1.0);
            }
            motorLinearSlide.setPower(linearSlidePower);
            clawServo.setPower(clawPower);
            rotateClawServo.setPower(rotateClawPower);



        }
    }
}
