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
public class AutoWithEncoders extends LinearOpMode 
{
    OpenCvWebcam webcam = null;
    private DcMotor turnTable;
    private DcMotor slidesMotor;
    private Servo claw;
    private Servo wrist;
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
        
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        examplepipeline pipeline = new examplepipeline();
        webcam.setPipeline(pipeline);
        
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            public void onOpened(){
                webcam.startStreaming(1280, 960,OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int errorCode){
            }
        });
        
        /*************************************** INITIALIZE ****************************************/
        double forearmPos = 0.45;
        int turnPos = 0;
        boolean scoreToggle = false;
        boolean scorePos = false;
        
        claw.setPosition(0.05);
        wrist.setPosition(0.19);
        clawForearm.setPosition(forearmPos); // down was 0.8 guys
        
        horizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        horizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (!isStarted() && !isStopRequested()){
        telemetry.addData("Realtime analysis", pipeline.getAnalysis());
        telemetry.update();
      
        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);
        }
            
        int snapshotAnalysis = pipeline.getAnalysis();
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();
        switch (snapshotAnalysis)
        {
            case 0:         /***************************** 0: RED *********************************/
            {
                doThis();
                mechEN(0,0,-90,1);
                mechEN(0,18,0,0.55);
                
                telemetry.addData("Color", "Red");
                telemetry.update();
                break;
            }
                
            case 1:         /***************************** 1: GREEN *********************************/
            {
                doThis();
                mechEN(0,0,-90,1);
                mechEN(0,-18,0,0.55);
                
                telemetry.addData("Color", "Green");
                telemetry.update();
                break;
            }
                
            case 2:         /***************************** 2: BLUE *********************************/
            {
                doThis();
                mechEN(0,0,-90,1);
                
                telemetry.addData("Color", "Blue");
                telemetry.update();
                break;
            }
        }
    }
    
public class examplepipeline extends OpenCvPipeline{
    //matrix to store color data
    Mat processing = new Mat();
    Mat processingB = new Mat();
    Mat centerCrop = new Mat();
    Mat centerCropB = new Mat();
    Mat output = new Mat();
    double percentValR;
    double percentValB;
    int position = 0;
    
    //sets the hue thresholds for the respective colors in HSV [hue, intensity, value]
    //RED
    Scalar lowB = new Scalar(90, 50, 50);
    Scalar highB = new Scalar(130,255,255);
    Scalar lowG = new Scalar(40, 20, 0);
    Scalar highG = new Scalar(80,255,255);
 
    public Mat processFrame(Mat input){
        telemetry.addLine("pipeline running");//camera intialized properly
        Imgproc.cvtColor(input, processing,Imgproc.COLOR_RGB2HSV);//convert to HSV color space
        Core.inRange(processing, lowG, highG, processing);//extract the desired color and make grayscale
        Rect centerRect = new Rect(800, 350, 400, 310);//creates bouding rectanlge
        centerCrop = processing.submat(centerRect);//creates subimage of given image
        percentValR = Core.sumElems(centerCrop).val[0] / centerRect.area() / 255;//calculates the percentage of white in the area
        centerCrop.release();//releases memory (optional, just an optimization)
        
        Imgproc.cvtColor(input, processingB,Imgproc.COLOR_RGB2HSV);//convert to HSV color space
        Core.inRange(processingB, lowB, highB, processingB);//extract the desired color and make grayscale
        centerCropB = processingB.submat(centerRect);//creates subimage of given image
        percentValB = Core.sumElems(centerCropB).val[0] / centerRect.area() / 255;//calculates the percentage of white in the area
        centerCropB.release();//releases memory (optional, just an optimization)
        
        processing.release();
        processingB.release();
     
        if (percentValR > 0.2 || percentValB > 0.2){
            if (percentValR > percentValB){
                telemetry.addData("Green", "Sleeve");
                Imgproc.putText(input, "Green",new Point(400, 350), 1 ,10,new Scalar (0.0,0.0,255.0), 2);
                //Imgproc.putText(input, Double.toString(percentValB),new Point(400, 350), 1 ,10,new Scalar (0.0,0.0,255.0), 2);
                Imgproc.rectangle(input, centerRect, new Scalar(0.0,0.0,255.0), 10);//draws bouding rectanlge with color and thickness
                telemetry.addData("Percent Value: ", percentValR);
                position = 1;
            }
            else{
                telemetry.addData("Blue", "Sleeve");
                Imgproc.putText(input, "Blue",new Point(400, 350), 1 ,10,new Scalar (0.0,0.0,255.0), 2);
                Imgproc.rectangle(input, centerRect, new Scalar(0.0,0.0,255.0), 10);//draws bouding rectanlge with color and thickness
                telemetry.addData("Percent Value: ", percentValB);
                position = 2;
            }
        }
        else{
            position = 0;
            Imgproc.rectangle(input, centerRect, new Scalar(0.0,0.0,255.0), 10);
        }
        input.copyTo(output);
        return output;
    }
    
    public int getAnalysis(){
        return position;
    }
}



    public void doThis(){
        mechEN(-35,0,0,0.5);
        mechEN(0,0,90,0.5);
        // highhhhhhhhhhh
        slidesMotor.setTargetPosition(-1350);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPower(1);
        sleep(800);
        turnTable.setPower(-1);
        sleep(900);
        turnTable.setPower(0);
        sleep(500); //600
        wrist.setPosition(0.40);
        wrist.setPosition(0.80);
        sleep(1000); //1200
        turnTable.setPower(0);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPower(1);
        turnTable.setPower(1);
        sleep(950);
        wrist.setPosition(0.19);
        turnTable.setPower(0);
        sleep(1000);
        
        for(int i=0; i<2; i++){
            // horizontal + claw system
            horizontalSlides.setTargetPosition(-1250);
            horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontalSlides.setPower(1);
            clawForearm.setPosition(0.85);
            
            sleep(1200);
            //if(colorSensor.red()>200 || colorSensor.blue()>200){
                claw.setPosition(0.6);
            //}
            sleep(500);
            clawForearm.setPosition(0.4);
            sleep(500);
            horizontalSlides.setTargetPosition(1);
            horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontalSlides.setPower(0.8);
            sleep(1100);
            claw.setPosition(0);
            sleep(500);
            clawForearm.setPosition(0.90);
            
            slidesMotor.setTargetPosition(-1350);
            slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesMotor.setPower(1);
            
            sleep(800);
            turnTable.setPower(-1);
            sleep(900);
            turnTable.setPower(0);
            sleep(500); //600
            wrist.setPosition(0.40);
            wrist.setPosition(0.80);
            sleep(1000); //1200
            turnTable.setPower(0);
            slidesMotor.setTargetPosition(0);
            slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidesMotor.setPower(1);
            turnTable.setPower(1);
            sleep(950);
            wrist.setPosition(0.19);
            turnTable.setPower(0);
            sleep(1000);
        }
        clawForearm.setPosition(0.55); // down was 0.8 guys

    }

   
    public void mechEN(double forMovement,double latMovement,double turn, double speed){
        int forwardSteps = (int)(forMovement * COUNTS_PER_INCH);
        int sideSteps = (int)(latMovement * COUNTS_PER_INCH);
        turn = ROBOT_DIAMETER_INCHES*Math.PI*(turn/180.0)*Math.PI/6.5;
        int turnSteps = (int)(turn * COUNTS_PER_INCH);
        
        
        int frontleftTargetPos   = frontleftDrive.getCurrentPosition() + (int)(forwardSteps + sideSteps - turnSteps);
        int frontrightTargetPos  = frontrightDrive.getCurrentPosition() + (int)(forwardSteps - sideSteps + turnSteps);
        int backleftTargetPos    = backleftDrive.getCurrentPosition() + (int)(forwardSteps - sideSteps - turnSteps);
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
    
    
    //math for forearm motor encoder
    public void slidesEN(double inchesHigh, double speed){
     
        double slidesSteps = (double)(((COUNTS_PER_DEGREE*81.72)*inchesHigh));
        int slidesTargetPos = (int)(slidesSteps);
        
        slidesMotor.setTargetPosition(slidesTargetPos);
        slidesMotor.setPower(Math.abs(speed));
      
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if(!opModeIsActive())
        {
            slidesMotor.setPower(0);
        }
    }
    
    
    public void horizontalEN(double inchesFar, double speed){
        double slidesSteps = (double)(((COUNTS_PER_DEGREE*81.72)*inchesFar));
        int slidesTargetPos = (int)(slidesSteps);
        
        horizontalSlides.setTargetPosition(slidesTargetPos);
        horizontalSlides.setPower(Math.abs(speed));
      
        horizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        if(!opModeIsActive())
        {
            horizontalSlides.setPower(0);
        }
    }
}
