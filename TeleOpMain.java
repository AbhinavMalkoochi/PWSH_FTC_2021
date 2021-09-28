

        package org.firstinspires.ftc.teamcode;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp

public class TeleOpMain extends LinearOpMode {

    DcMotor motorFL, motorBL, motorBR, motorFR, intakeHex;


    public void runOpMode() throws InterruptedException{

        //Mapping Motors
        motorFL=hardwareMap.get(DcMotor.class,"FrontLeftMotor");
        motorBL=hardwareMap.get(DcMotor.class,"BackLeftMotor");
        motorBR=hardwareMap.get(DcMotor.class,"BackRightMotor");
        motorFR=hardwareMap.get(DcMotor.class,"FrontRightMotor");
        intakeHex = hardwareMap.get(DcMotor.class, "IntakeHexMotor");

        //Setting motor behavior when power is zero
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeHex.setPower(0.0);

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y*0.3;
            double x = gamepad1.left_stick_x*1.1;
            double dx = gamepad1.right_stick_x*0.3;
            double hexP = gamepad.a*0.3;

            double motorFLPower = (y + x + dx);
            double motorBLPower = (y - x + dx);
            double motorFRPower = (y - x - dx);
            double motorBRPower = (y + x - dx);
            double motorHEXPower = (hexP); 
            
            motorFL.setPower(motorFLPower);
            motorFR.setPower(motorFRPower);
            motorBL.setPower(motorBLPower);
            motorBR.setPower(motorBRPower);
            intakeHex.setPower(hexP); 


        }

    }




}
