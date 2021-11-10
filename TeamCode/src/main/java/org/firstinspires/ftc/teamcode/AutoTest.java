
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.*;

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
public class AutoTest extends LinearOpMode{
    //change values
    public static final double REV_COUNT= 560;//counts per revolution
    public static final double DIAMETER = 2.95276;
    public static final double GEAR_RATIO = 5;
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
        duckMotor = hardwareMap.get(DcMotor.class,"DuckMotor");
        motorFLSecond = hardwareMap.get(DcMotor.class,"FrontLeftMotorSecond");
        intakeHex = hardwareMap.get(DcMotor.class, "IntakeHexMotor");
        motorLinearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        rotateClawServo = hardwareMap.get(Servo.class,"RotateServo");
        clawServo = hardwareMap.get(Servo.class,"ClawServo");
        intakeHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        waitForStart();
        driveForward(5,1);
        sleep(1000);
        setSpeed(0);
        driveBackward(5,1);
        sleep(1000);
        setSpeed(0);
        strafeLeft(5,1);
        sleep(1000);
        setSpeed(0);
        strafeRight(5,1);
        sleep(1000);
        setSpeed(0);

        while(opModeIsActive()){

            // carouselPosition();

            //sleep(1000);


        }
    }

    public void driveForward(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        motorFL.setTargetPosition(motorFL.getCurrentPosition()-distance);
        motorFR.setTargetPosition(motorFR.getCurrentPosition()+distance);
        motorBL.setTargetPosition(motorBL.getCurrentPosition()-distance);
        motorBR.setTargetPosition(motorBR.getCurrentPosition()+distance);
        setSpeed(speed);

    }
    public void driveBackward(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        //motor 0 - FL, motor 1 - BL, motor 2 - FR, motor 3 - BR
        motorFL.setTargetPosition(motorFL.getCurrentPosition()+distance);
        motorFR.setTargetPosition(motorFR.getCurrentPosition()-distance);
        motorBL.setTargetPosition(motorBL.getCurrentPosition()+distance);
        motorBR.setTargetPosition(motorBR.getCurrentPosition()-distance);
        setSpeed(speed);
    }
    public void strafeLeft(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        motorFL.setTargetPosition(motorFL.getCurrentPosition()-distance);
        motorFR.setTargetPosition(motorFR.getCurrentPosition()+distance);
        motorBL.setTargetPosition(motorBL.getCurrentPosition()+distance);
        motorBR.setTargetPosition(motorBR.getCurrentPosition()-distance);
        setSpeed(speed);
    }
    public void strafeRight(int distance, double speed){
        distance = (int)(Math.round(distance * INCHES_ROTATION * 0.7));

        motorFL.setTargetPosition(motorFL.getCurrentPosition()+distance);
        motorFR.setTargetPosition(motorFR.getCurrentPosition()-distance);
        motorBL.setTargetPosition(motorBL.getCurrentPosition()-distance);
        motorBR.setTargetPosition(motorBR.getCurrentPosition()+distance);
        setSpeed(speed);
    }
    /*
    public void linearSlideExtendLevel1(){
        motorLinearSlide.setPower(-1.0);
        sleep(1000);
        motorLinearSlide.setPower(1.0);
        sleep(1000);
        motorLinearSlide.setPower(0.0);
        rotateClawServo.setPosition(1);
        clawServo.setPosition(1.0);
    }
    */

    public void carouselPosition(){
        turn(180,1);
        //sleep(1000);
        motorLinearSlide.setPower(1);
        //sleep(1000);
        rotateClawServo.setPosition(0.0);
        //sleep(1000);
        clawServo.setPosition(1.0);
        //sleep(1000);
        turn(-90,1);
        //sleep(1000);
        driveForward(15,1);
        //sleep(1000);
        intakeHex.setPower(1);
        turn(-90,1000);
        driveForward(2,1);
        //sleep(1000);
        turn(-90,1000);
        //sleep(1000);
        driveForward(30,1);
        ///sleep(1000);
        setSpeed(0);
        rotate90();
        setSpeed(0);

    }
    public void setSpeed(double speed){

        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    public void rotate90(){
        motorFL.setPower(-1);
        motorBL.setPower(-1);
        motorFR.setPower(1);
        motorBR.setPower(1);

        setSpeed(0);
    }
    public void turn(int degrees, double speed){
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        double angleTurn = currAngle();
        if(angleTurn!=degrees && angleTurn>0  ){
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
    }
}