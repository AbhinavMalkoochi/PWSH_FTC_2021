package org.firstinspires.ftc.teamcode;//Num of inches to move * (circumference of wheel/tick count)*gear ratio(1)
/*
    TODO
    BEFORE QUAL
    AUTO: 2 ducks and park
    TELEOP: have slide control working
    AFTER QUAL
    fix drive
    roadrunner
    robot class get this done first
    opencv
    automation

 */

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.*;
import org.firstinspires.ftc.teamcode.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.Console;
import java.util.Locale;

@Autonomous
public class BlueAuto extends LinearOpMode{
    //change values
    public static final int MAX = 220;
    public static final int MIN = -700;
    public static final double REV_COUNT= 560;//counts per revolution
    public static final double DIAMETER = 2.95276;
    public static final double GEAR_RATIO = 5;
    public static int ANGLES = 90;
    public static double TICK_ANGLE = 15;
    private static final double INCHES_ROTATION = (REV_COUNT * GEAR_RATIO) / (DIAMETER * Math.PI);// counts per revolution
    DcMotor motorFL, motorBL, motorBR, motorFR, motorLinearSlide, intakeHex, motorFLSecond, duckMotor;
    Servo rotateClawServo , clawServo;
    BNO055IMU imu;
    Orientation angles = new Orientation();
    double finalAngle;
    Acceleration gravity;

    public void runOpMode() throws InterruptedException{

        motorFL = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        motorBL = hardwareMap.get(DcMotor.class,"BackLeftMotor");
        motorBR = hardwareMap.get(DcMotor.class,"BackRightMotor");
        motorFR = hardwareMap.get(DcMotor.class,"FrontRightMotor");
        motorFLSecond = hardwareMap.get(DcMotor.class,"FrontLeftMotorSecond");

        duckMotor = hardwareMap.get(DcMotor.class,"DuckMotor");
        intakeHex = hardwareMap.get(DcMotor.class, "IntakeHexMotor");
        motorLinearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        rotateClawServo = hardwareMap.get(Servo.class,"RotateServo");
        clawServo = hardwareMap.get(Servo.class,"ClawServo");
        intakeHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // motorFL.setDirection(DcMotor.Direction.REVERSE);
         // motorBR.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //setpower
        //reset
        //target
        //run
        setSpeed(0);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFLSecond.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setTargetPosition(motorFL.getCurrentPosition());
        motorBL.setTargetPosition(motorBL.getCurrentPosition());
        motorFR.setTargetPosition(motorFR.getCurrentPosition());
        motorBR.setTargetPosition(motorFR.getCurrentPosition());
        motorFLSecond.setTargetPosition(motorFR.getCurrentPosition());

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFLSecond.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        waitForStart();
        // moveForward(2,1,1);
/*
        clawServo.setPosition(0);
        moveBackward(2,1,1);
        sleep(2000);
        setSpeed(0);
        motorLinearSlide.setTargetPosition(motorLinearSlide.getCurrentPosition()+500);
        motorLinearSlide.setPower(1);
        sleep(3000);
        motorLinearSlide.setPower(0);
        rotateClawServo.setPosition(0);
        sleep(1000);
        clawServo.setPosition(1);
        sleep(1000);
        rotateClawServo.setPosition(1);
        rotate(270);
        moveForward(3,1.0,0.7);
        sleep(1000);
        setSpeed(0);
        rotate(-30);
        duckMotor.setPower(1);
        sleep(2000);
        duckMotor.setPower(0);
        duckMotor.setPower(1);
        sleep(2000);
        duckMotor.setPower(0);
        rotate(30);
        moveBackward(2,1,0.7);
        rotate(-90);
        moveForward(20,1,0.7);

 */
        /*
        int target = (int)(Math.round(1 * INCHES_ROTATION ));
        motorFL.setTargetPosition(motorFL.getCurrentPosition() + target); motorBL.setTargetPosition(motorBL.getCurrentPosition() - target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - target); motorBR.setTargetPosition(motorBR.getCurrentPosition() + target);
        setSpeed(1);
        sleep( 5000);
        setSpeed(0);
        duckMotor.setPower(1);
        sleep(5000);
        duckMotor.setPower(0);
        rotate(-90);
        moveForward(20,1,1);
*/

        int target = (int)(Math.round(1 * INCHES_ROTATION ));
        motorFL.setTargetPosition(motorFL.getCurrentPosition() + target); motorBL.setTargetPosition(motorBL.getCurrentPosition() - target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - target); motorBR.setTargetPosition(motorBR.getCurrentPosition() + target);
        setSpeed(1);
        sleep(2000);
        setSpeed(0);
        duckMotor.setPower(1);
        sleep(3000);
        duckMotor.setPower(0);
        target = (int)(Math.round(4.5 * INCHES_ROTATION ));
        motorFL.setTargetPosition(motorFL.getCurrentPosition() - target); motorBL.setTargetPosition(motorBL.getCurrentPosition() - target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() + target); motorBR.setTargetPosition(motorBR.getCurrentPosition() + target);
        setSpeed(1);
        sleep(2000);
        setSpeed(0);
        strafeRight(1.5,5,1);
        sleep(1000);
        setSpeed(0);

        /*
        PARK
        rotate(270);
        moveForward(10,1,1);
        sleep(2000);
        setSpeed(0);

         */
        while(opModeIsActive()){



        }
    }


    public void moveForward(double inches, double speed, double coeff) {

        //resetMotors();

        int target = (int)(Math.round(inches * INCHES_ROTATION * coeff));

        motorFL.setTargetPosition(motorFL.getCurrentPosition() + target); motorBL.setTargetPosition(motorBL.getCurrentPosition() + target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() + target); motorBR.setTargetPosition(motorBR.getCurrentPosition() + target);
        motorFLSecond.setTargetPosition(motorBR.getCurrentPosition() + target);
       // runMotors(speed);
/*
        while(opModeIsActive() && motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy()) {
            setSpeed(speed);

        }
*/
        setSpeed(speed);
        //stopMotors();

    }

    public void moveBackward(double inches, double speed, double coeff) {

        //resetMotors();

        int target = (int)(Math.round(inches * INCHES_ROTATION * coeff));

        motorFL.setTargetPosition(motorFL.getCurrentPosition() + target); motorBL.setTargetPosition(motorBL.getCurrentPosition() + target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - target); motorBR.setTargetPosition(motorBR.getCurrentPosition() - target);
        runMotors(speed);

        while(opModeIsActive() && motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy()) {
            setSpeed(speed);
        }
        stopMotors();

    }

    public void strafeLeft(double inches, double speed, double coeff) {

        //resetMotors();

        int target = (int)(Math.round(inches * INCHES_ROTATION * coeff));

        motorFL.setTargetPosition(motorFL.getCurrentPosition() + target); motorBL.setTargetPosition(motorBL.getCurrentPosition() - target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() + target); motorBR.setTargetPosition(motorBR.getCurrentPosition() - target);
        runMotors(speed);

        while(opModeIsActive() && motorFL.isBusy() && motorBL.isBusy() && motorFR.isBusy() && motorBR.isBusy()) {
            setSpeed(speed);
        }

        stopMotors();

    }

    public void strafeRight(double inches, double speed, double coeff) {

        //resetMotors();

        int target = (int)(Math.round(inches * INCHES_ROTATION * coeff));
        motorFLSecond.setTargetPosition(motorFL.getCurrentPosition() + target);
        motorFL.setTargetPosition(motorFL.getCurrentPosition() + target); motorBL.setTargetPosition(motorBL.getCurrentPosition() - target);
        motorFR.setTargetPosition(motorFR.getCurrentPosition() - target); motorBR.setTargetPosition(motorBR.getCurrentPosition() + target);
        setSpeed(speed);


    }
    /*
        public void //resetMotors() {
            motorFL.setTargetPosition(0);
            motorFR.setTargetPosition(0);
            motorBL.setTargetPosition(0);
            motorBR.setTargetPosition(0);

            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(50);
        }

     */
    public void runMotors(double speed) {
        motorFL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        motorFLSecond.setTargetPosition(0);

        motorFL.setPower(speed); motorBL.setPower(speed);
        motorFR.setPower(speed); motorBR.setPower(speed);
        motorFLSecond.setPower(speed);
        motorFLSecond.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION); motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION); motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void stopMotors() {
        motorFLSecond.setPower(0);
        motorFL.setPower(0); motorBL.setPower(0);
        motorFR.setPower(0); motorBR.setPower(0);

    }
    public void LinearSlideL1(){
        if(motorLinearSlide.getCurrentPosition()<MAX){
            motorLinearSlide.setTargetPosition(200);
            motorLinearSlide.setPower(1);
        }else{
            motorLinearSlide.setPower(0);
        }
        sleep(500);
        motorLinearSlide.setPower(0);
        rotateClawServo.setPosition(-1);
        sleep(100);
        clawServo.setPosition(1.0);
        sleep(1000);
    }
    public void LinearSlideL2(){
        if(motorLinearSlide.getCurrentPosition()<MAX){
            motorLinearSlide.setTargetPosition(400);
            motorLinearSlide.setPower(1);
        }else{
            motorLinearSlide.setPower(0);
        }
        sleep(1000);
        motorLinearSlide.setPower(0);
        rotateClawServo.setPosition(-1);
        sleep(100);
        clawServo.setPosition(1.0);
        sleep(1000);
    }
    public void LinearSlideL3(){
        if(motorLinearSlide.getCurrentPosition()<MAX){
            motorLinearSlide.setTargetPosition(800);
            motorLinearSlide.setPower(1);
        }else{
            motorLinearSlide.setPower(0);
        }
        sleep(3000);
        motorLinearSlide.setPower(0);
        rotateClawServo.setPosition(-1);
        sleep(1000);
        clawServo.setPosition(1.0);
        sleep(1000);
    }

    public void setSpeed(double speed){

        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        motorBR.setPower(speed);
        motorFLSecond.setPower(speed);

    }



    public double currAngle(){

        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.DEGREES);
        double deltaAngle = angles.thirdAngle - angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        finalAngle += deltaAngle;
        angles = angle;
        return finalAngle;

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void rotate(int angle){
        //resetMotors();
        motorFL.setTargetPosition((int)(motorFL.getCurrentPosition()+angle*TICK_ANGLE));
        motorBL.setTargetPosition((int)(motorFL.getCurrentPosition()+angle*TICK_ANGLE));
        motorFR.setTargetPosition((int)(motorFL.getCurrentPosition()+angle*TICK_ANGLE));
        motorBR.setTargetPosition((int)(motorFL.getCurrentPosition()+angle*TICK_ANGLE));
        setSpeed(1);
    }
    public void turn(int degrees, double speed){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        double angleTurn = currAngle();
        if(angleTurn!=degrees && angleTurn > 0){
            motorFL.setPower(speed);
            motorBL.setPower(speed);
            motorFR.setPower(speed);
            motorBR.setPower(speed);
        }else if(angleTurn!=degrees && angleTurn<0){
            motorFL.setPower(-speed);
            motorBL.setPower(-speed);
            motorFR.setPower(speed);
            motorBR.setPower(speed);
        }
        setSpeed(0);
        //resetMotors();
    }
}