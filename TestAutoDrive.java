// power play season
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestAutoDrive extends LinearOpMode 
{
    OpenCvWebcam webcam = null;
    private DcMotor turnTable;
    private DcMotor slidesMotor;
    private Servo claw;
    private Servo clawWrist;
    private Servo clawForearm;
    private ElapsedTime runtime = new ElapsedTime();

    
    static final double COUNTS_PER_MOTOR_REV    = 751.1 ;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_DEGREE       = COUNTS_PER_MOTOR_REV / 360;
    
    private DcMotor frontleftDrive = null;
    private DcMotor frontrightDrive = null;
    private DcMotor backleftDrive = null;
    private DcMotor backrightDrive = null;
    private DcMotor horizontalSlides = null;
    private ColorSensor colorSensor;
    private BNO055IMU imu;
    double  position = (MAX_POS - MIN_POS) / 2;
    
    static final double INCREMENT   =   5/90.0;              // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;                  // period of each cycle
    static final double MAX_POS     =   60.0/90.0;           // Maximum rotational position
    static final double MIN_POS     =   0;                   // Minimum rotational position
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;      // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 3.75 ;     // For figuring circumference
    static final double ROBOT_DIAMETER_INCHES   = 20;
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    boolean toggle1 = false;
    boolean toggle2 = false;
    boolean motorPos = false;
    boolean servoPos = false;
    
    double forearmPos = 0.5;
   
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize the hardware variables.
        frontleftDrive  = hardwareMap.get(DcMotor.class, "frontleftDrive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontrightDrive");
        backleftDrive  = hardwareMap.get(DcMotor.class, "backleftDrive");
        backrightDrive = hardwareMap.get(DcMotor.class, "backrightDrive");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        horizontalSlides = hardwareMap.get(DcMotor.class, "horizontalSlides");
        turnTable = hardwareMap.get(DcMotor.class, "turnTable");
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        claw = hardwareMap.get(Servo.class, "claw");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        clawForearm = hardwareMap.get(Servo.class, "clawForearm");
        
        // set motors directions
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);
        horizontalSlides.setDirection(DcMotor.Direction.REVERSE);
        slidesMotor.setDirection(DcMotor.Direction.FORWARD);
        turnTable.setDirection(DcMotor.Direction.FORWARD);
        
        // reset encoder
        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // start encoders
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        /***************************** INITIALIZE *********************************/
        double forearmPos = 0.5;
        claw.setPosition(0.1);
        clawWrist.setPosition(0.60);
        clawForearm.setPosition(forearmPos); // down was 0.8 guys
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!isStarted() && !isStopRequested()){
        telemetry.update();
      
        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);
        }
        mechEN(40,0,0, 0.5);

    }


    public void mechEN(double forMovement,double latMovement,double turn, double speed){
        int forwardSteps = (int)(forMovement * COUNTS_PER_INCH);
        int sideSteps = (int)(latMovement * COUNTS_PER_INCH);
        turn = ROBOT_DIAMETER_INCHES*Math.PI*(turn/180.0)*Math.PI/6.5;
        int turnSteps = (int)(turn * COUNTS_PER_INCH);
        
        
        int frontleftTargetPos   = frontleftDrive.getCurrentPosition() + (int)(forwardSteps + sideSteps - turnSteps);
        int frontrightTargetPos  = frontrightDrive.getCurrentPosition() + (int)(forwardSteps - sideSteps + turnSteps);
        int backleftTargetPos    = backleftDrive.getCurrentPosition() + (int)(forwardSteps - sideSteps + turnSteps);
        int backrightTargetPos   = backrightDrive.getCurrentPosition() + (int)(forwardSteps + sideSteps + turnSteps);

        frontleftDrive.setTargetPosition(frontleftTargetPos);
        frontrightDrive.setTargetPosition(frontrightTargetPos);
        backleftDrive.setTargetPosition(backleftTargetPos);
        backrightDrive.setTargetPosition(backrightTargetPos);
        
        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontleftDrive.setPower(Math.abs(speed));
        frontrightDrive.setPower(Math.abs(speed));
        backleftDrive.setPower(Math.abs(speed));
        backrightDrive.setPower(Math.abs(speed));
        
        while (opModeIsActive() && (frontleftDrive.isBusy() && frontrightDrive.isBusy() && backleftDrive.isBusy() && backrightDrive.isBusy())) {
        }
        
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
    }
   
}
