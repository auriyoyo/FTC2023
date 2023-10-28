package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Final_TeleOp", group="Linear Opmode")

public class FinalTeleop extends LinearOpMode {
    
    private DcMotor turnTable;
    private DcMotor slidesMotor;
    private Servo claw;
    private Servo wrist;
    private Servo clawForearm;
    
    static final double COUNTS_PER_MOTOR_REV    = 751.1 ;    // eg: TETRIX Motor Encoder
    static final double COUNTS_PER_DEGREE       = COUNTS_PER_MOTOR_REV / 360;
    
    private ElapsedTime runtime = new ElapsedTime();
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
        wrist = hardwareMap.get(Servo.class, "passiveWrist");
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
        
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        // initialize servos
        
        double forearmPos = 0.2;
        int turnPos = 0;
        boolean scoreToggle = false;
        boolean scorePos = false;
        
        wrist.setPosition(0.15);
        clawForearm.setPosition(forearmPos); // down was 0.8 guys
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
     
        
        while (opModeIsActive()) {
            
            /* DRIVE TRAIN CODE */
            double frontleftPower;
            double frontrightPower;
            double backleftPower;
            double backrightPower;
            
            double turnTableLeft;
            
            // go faster with left trigger and slower with right trigger
            double maxPower;
            if (gamepad2.right_bumper){
                maxPower = 0.2;
            }
            else {
                maxPower = 0.4;
            }
            telemetry.addData("heading", -imu.getAngularOrientation().firstAngle);
            telemetry.update();
            
            //gamepad 1
            double up = gamepad1.left_stick_y;
            double right = -gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_x;
            
            robotCentric(up, right, turn, maxPower);
            //fieldCentric(up, -right, turn, maxPower);
            
            
            /* CONTROLLER 2 */
            // FOREARM
            if(gamepad2.right_trigger>0.3){
                forearmPos -= 0.005;
                clawForearm.setPosition(forearmPos);
                telemetry.addData("Position", forearmPos);
            }
            else if(gamepad2.left_trigger>0.3){
                forearmPos += 0.005;
                clawForearm.setPosition(forearmPos);
                telemetry.addData("Position", forearmPos);
            }
            // CLAW
            if(clawForearm.getPosition() != 0.4){
                if(colorSensor.red()>800 || colorSensor.blue()>800){
                    claw.setPosition(0.55);
                }
            }
            if(gamepad2.right_bumper){
                claw.setPosition(0.1);
            }
            if(gamepad2.left_bumper){
                claw.setPosition(0.55);
            }
            // AUTOMATION
            if(gamepad2.y){  // up
                horizontalSlides.setTargetPosition(1);
                forearmPos = 0.17;
                clawForearm.setPosition(forearmPos);
            }
            if(gamepad2.a){ // down
                forearmPos = 0.67;
                clawForearm.setPosition(forearmPos);
                claw.setPosition(0.1);
            }
            // HORIZONTAL SLIDES 
            horizontalSlides.setPower(gamepad2.left_stick_y);
            /*
            if (gamepad2.left_stick_y > 0){
                horizontalSlides.setTargetPosition(horizontalSlides.getCurrentPosition()+(int)(100*gamepad2.left_stick_y));
                horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlides.setPower(1);
            }
            else if (gamepad2.left_stick_y < 0){
                horizontalSlides.setTargetPosition(horizontalSlides.getCurrentPosition()+(int)(100*gamepad2.left_stick_y));
                horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlides.setPower(1);
            }
            
            else if(horizontalSlides.getCurrentPosition() >= 10000){
                horizontalSlides.setTargetPosition(9900);
                horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlides.setPower(1);
            }
            
            else if(horizontalSlides.getCurrentPosition() <= 5){
                horizontalSlides.setTargetPosition(10);
                horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalSlides.setPower(1);               
            }
            */
            
            
            
            
            /* CONTROLLER 1 */
            // TURNTABLE
            
            turnTable.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            
            
            // VERTICAL SLIDES
            if (gamepad1.dpad_down){
                slidesMotor.setTargetPosition(slidesMotor.getCurrentPosition()+200);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
                telemetry.addData("Position", slidesMotor.getCurrentPosition());
            }
            else if(gamepad1.dpad_up){
                slidesMotor.setTargetPosition(slidesMotor.getCurrentPosition()-200);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
                telemetry.addData("Position", slidesMotor.getCurrentPosition());
                
            }
            // VERTICAL SLIDES SET POSITIONS
            if(gamepad1.y){
                slidesMotor.setTargetPosition(-1350);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
            }
            if(gamepad1.x){
                slidesMotor.setTargetPosition(-550);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
            }
            if(gamepad1.a){
                slidesMotor.setTargetPosition(0);
                slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesMotor.setPower(1);
            }
            // wrist
            if(gamepad1.b && !scoreToggle){
                if(scorePos){
                    wrist.setPosition(0.85);
                    sleep(1000);
                    wrist.setPosition(0.17);
                }
                else{
                    wrist.setPosition(0.45);
                }
                scorePos = !scorePos;
                scoreToggle = true;
            }
            else if(!gamepad1.b&&scoreToggle){
                scoreToggle = false;
            }
            
        }
    }
    
    public void robotCentric(double up, double right, double turn, double maxPower){
        double frontleftPower    = Range.clip(up +right -turn, -maxPower, maxPower);
        double frontrightPower   = Range.clip(up -right +turn, -0.3, 0.3);
        double backleftPower     = Range.clip(up -right -turn, -maxPower, maxPower);
        double backrightPower    = Range.clip(up +right +turn, -maxPower, maxPower);
        
        frontleftDrive.setPower(frontleftPower);
        frontrightDrive.setPower(frontrightPower);
        backleftDrive.setPower(backleftPower);
        backrightDrive.setPower(backrightPower);
        
    }
    
    public void fieldCentric(double y, double x, double turn, double maxPower){
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x*Math.cos(botHeading) - y*Math.sin(botHeading);
        double rotY = x*Math.sin(botHeading) + y*Math.cos(botHeading);
        
        double frontleftPower    = Range.clip(rotY +rotX -turn, -maxPower, maxPower);
        double frontrightPower   = Range.clip(rotY -rotX +turn, -0.3, 0.3);
        double backleftPower     = Range.clip(rotY -rotX -turn, -maxPower, maxPower);
        double backrightPower    = Range.clip(rotY +rotX +turn, -maxPower, maxPower);
        
        frontleftDrive.setPower(frontleftPower);
        frontrightDrive.setPower(frontrightPower);
        backleftDrive.setPower(backleftPower);
        backrightDrive.setPower(backrightPower);
        
    }
}
