package org.firstinspires.ftc.teamcode;
import java.util.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.io.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TESTWOAH", group="Linear Opmode")

public class TestingMotorDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotor motor;
    
    @Override
    public void runOpMode() {
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor  = hardwareMap.get(DcMotorEx.class, "motor");
        //spinningPlane = hardwareMap.get(DcMotor.class, "spinningPlane");
        //testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        
        //set direction
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        //spinningPlane.setDirection(DcMotor.Direction.FORWARD);
        //testMotor.setDirection(DcMotor.Direction.FORWARD);
        
        //testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            
            //testMotor.setPower(gamepad1.right_stick_y);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            motor.setPower(1);
        }
        }
}
