package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Servo_Test", group="Linear Opmode")

public class ServoTest extends LinearOpMode {
    private CRServo cServo;
    private DcMotor intake;
    private ElapsedTime runtime = new ElapsedTime();
    public void runOpMode(){
        
        //cServo = hardwareMap.get(CRServo.class, "cServo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            intake.setPower(0.2);
            //cServo.setPower(1);
        }
    }
}
