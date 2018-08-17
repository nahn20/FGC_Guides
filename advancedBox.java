package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="advancedBox", group="Box")
public class advancedBox extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();

    private Servo tiltServo = null;
    private Servo releaseServo = null;
    private Servo dropServo = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor ballSensor = null;
    private DcMotor left_cannon = null;
    private DcMotor right_cannon = null;
    private Servo cannonServo = null;
    private CRServo bumpServo = null;
    private CRServo bumpServo2 = null;
    private DcMotor spinnyMotor = null;
    
    double slowTiltPos = 999;

    double servoCannonPos = .893;
    double servoLoadPos = .18;
    double servoDefaultPos = .48;

    int nextBall = 1;

    boolean loaded = false;
    
    @Override
    public void runOpMode() {
        tiltServo = hardwareMap.servo.get("tiltServo");
        releaseServo = hardwareMap.servo.get("releaseServo");
        dropServo = hardwareMap.servo.get("dropServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        ballSensor = hardwareMap.get(DistanceSensor.class, "ballSensor");
        left_cannon = hardwareMap.dcMotor.get("left_cannon");
        right_cannon = hardwareMap.dcMotor.get("right_cannon");
        cannonServo = hardwareMap.servo.get("cannonServo");
        bumpServo = hardwareMap.crservo.get("bumpServo");
        bumpServo2 = hardwareMap.crservo.get("bumpServo2");
        spinnyMotor = hardwareMap.dcMotor.get("spinnyMotor");
        
        left_cannon.setDirection(DcMotor.Direction.REVERSE); 
        left_cannon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        right_cannon.setDirection(DcMotor.Direction.FORWARD); 
        right_cannon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        tiltServo.setPosition(servoDefaultPos);
        dropServo.setPosition(.563);
        cannonServo.setPosition(.52);
        
        useMap1.undefinedCD2 = runtime.milliseconds();
        
        while(!opModeIsActive()){}
        useMap1.undefinedCD1 = runtime.milliseconds();
        useMap1.x = runtime.milliseconds();
        while(opModeIsActive()){
            telemetry.addData("Read this number here", colorSensorCheck());
            updateKeys();
            
            sendNextBall();
            cannon();
            
            setNextBall();
            
            gateControl();
            
            bumpControl();
            info();
            
            telemetry.update();
        }
    }
    public void sendNextBall(){
        //UndefinedCD1: Drop trigger
        //UndefinedCD2: Delay for next cycle and for ball drop time
        int colorSensorCheck = colorSensorCheck();
        if(colorSensorCheck != 0){
            dropServo.setPosition(.7);
            if(cdCheck(useMap1.undefinedCD2, 800) && toggleMap1.y){
                if(!loaded && colorSensorCheck == nextBall && !toggleMap1.x && cdCheck(useMap1.x, 500)){
                    tiltServo.setPosition(servoLoadPos);
                }
                else if(cdCheck(useMap1.undefinedCD5, 2000)){ //Wait 2 seconds after a ball hasn't been in launcher
                    tiltServo.setPosition(servoCannonPos);
                }
                if(cdCheck(useMap1.undefinedCD1, 8000)){ //Returns to middle if stuck probably
                    tiltServo.setPosition(servoDefaultPos);
                    useMap1.undefinedCD1 = runtime.milliseconds();
                }
            }
        }
        else if(colorSensorCheck == 0){
            if(Math.abs(tiltServo.getPosition() - servoCannonPos) < 0.01){
                tiltServo.setPosition(servoDefaultPos);
                useMap1.undefinedCD1 = runtime.milliseconds();
            }
            else if(Math.abs(tiltServo.getPosition() - servoLoadPos) < 0.01){
                tiltServo.setPosition(servoDefaultPos);
                useMap1.undefinedCD1 = runtime.milliseconds();
                loaded = true;
                if(nextBall == 1){
                    nextBall = 2;
                }
                else if(nextBall == 2){
                    nextBall = 1;
                }
            }
            else if(cdCheck(useMap1.undefinedCD1, 500)){
                dropServo.setPosition(.563);
                useMap1.undefinedCD2 = runtime.milliseconds();
                useMap1.undefinedCD1 = runtime.milliseconds();
            }
            
        }
    }
    public void cannon(){
        if(!toggleMap1.b){
            if(!cdCheck(useMap1.undefinedCD4, 2400) && cdCheck(useMap1.undefinedCD4, 1200)){ //Stops in case ball misses, no push out
                left_cannon.setPower(0);
                right_cannon.setPower(0);
            }
            else{
                left_cannon.setPower(1);
                right_cannon.setPower(1);
            }
        }
        else{
            left_cannon.setPower(0);
            right_cannon.setPower(0);
        }
        if(ballSensor.getDistance(DistanceUnit.CM) <= 8 && cdCheck(useMap1.undefinedCD4, 2000)){
            useMap1.undefinedCD4 = runtime.milliseconds();
        }
        if(((cdCheck(useMap1.undefinedCD4, 500) && !cdCheck(useMap1.undefinedCD4, 1000)) || gamepad1.a) && cdCheck(useMap1.a, 500)){
            cannonServo.setPosition(.3);
            useMap1.a = runtime.milliseconds();
        }
        else if(cdCheck(useMap1.a, 500)){
            cannonServo.setPosition(.52);
        }
        if(ballSensor.getDistance(DistanceUnit.CM) <= 10){
            useMap1.undefinedCD5 = runtime.milliseconds();
        }
    }
    public int colorSensorCheck(){ //0 = unknown, 1 = red, 2 = blue
        int color = 0;
        int totalCount = 0;
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double scale_factor = 255;
        Color.RGBToHSV((int) (colorSensor.red() * scale_factor),
                (int) (colorSensor.green() * scale_factor),
                (int) (colorSensor.blue() * scale_factor),
                hsvValues);
        telemetry.addData("Color Values", hsvValues[0]);
        for(int i = 0; i < 50; i++){
            if (hsvValues[0] <= 250 && hsvValues[0] >= 130) {
                color = 2;
            }
            else if (hsvValues[0] <= 5 || (hsvValues[0] >= 300 && hsvValues[0] <= 400)) {
                color = 1;
            }
            totalCount += color;
        }
        color = totalCount/50;
        if(Math.abs(color - 1) < 0.1){
            color = 1;
        }
        else if(Math.abs(color - 2) < 0.1){
            color = 2;
        }
        else{
            color = 0;
        }
        return color;
    }
    public void setNextBall(){
        if(gamepad1.right_bumper){
            nextBall = 2;
        }
        if(gamepad1.left_bumper){
            nextBall = 1;
        }
    }
    public void gateControl(){
        if(toggleMap1.x){
            releaseServo.setPosition(.2);
            loaded = false;
        }
        else if(!toggleMap1.x){
            releaseServo.setPosition(1);
        }
    }
    public void bumpControl(){
        if(!toggleMap1.dpad_right){
            bumpServo.setPower(1);
        }
        else{
            bumpServo.setPower(0);
        }
        if(!toggleMap1.dpad_up){
            bumpServo2.setPower(1);
        }
        else{
            bumpServo2.setPower(0);
        }
        if(!toggleMap1.dpad_left){
            spinnyMotor.setPower(-1);
        }
        else{
            spinnyMotor.setPower(0);
        }
    }
    public void updateKeys(){ 
        if(gamepad1.a && cdCheck(useMap1.a, 1000)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
        }
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
        }
        if(gamepad1.x && cdCheck(useMap1.x, 500)){
            toggleMap1.x = toggle(toggleMap1.x);
            useMap1.x = runtime.milliseconds();
        }
        if(gamepad1.y && cdCheck(useMap1.y, 500)){
            toggleMap1.y = toggle(toggleMap1.y);
            useMap1.y = runtime.milliseconds();
        }
        if(gamepad1.dpad_right && cdCheck(useMap1.dpad_right, 500)){
            toggleMap1.dpad_right = toggle(toggleMap1.dpad_right);
            useMap1.dpad_right = runtime.milliseconds();
        }
        if(gamepad1.dpad_up && cdCheck(useMap1.dpad_up, 500)){
            toggleMap1.dpad_up = toggle(toggleMap1.dpad_up);
            useMap1.dpad_up = runtime.milliseconds();
        }
        if(gamepad1.dpad_left && cdCheck(useMap1.dpad_left, 500)){
            toggleMap1.dpad_left = toggle(toggleMap1.dpad_left);
            useMap1.dpad_left = runtime.milliseconds();
        }
        if(gamepad1.dpad_down && cdCheck(useMap1.dpad_down, 500)){
            if(toggleMap1.dpad_right == toggleMap1.dpad_up && toggleMap1.dpad_up == toggleMap1.dpad_left){
                toggleMap1.dpad_down = toggleMap1.dpad_up;
            }
            toggleMap1.dpad_down = toggle(toggleMap1.dpad_down);
            toggleMap1.dpad_right = toggleMap1.dpad_down;
            toggleMap1.dpad_up = toggleMap1.dpad_down;
            toggleMap1.dpad_left = toggleMap1.dpad_down;
            useMap1.dpad_down = runtime.milliseconds();
        }
        
    }
    public void info(){
        telemetry.addData("(Y) Running Ball Sorter?", toggleMap1.y);
        telemetry.addData("(B) Running Cannon?", toggleMap1.b);
        telemetry.addData("Is Loaded?", loaded);
        String ballColor = "Bleh";
        if(nextBall = 1){
            ballColor = "Red";
        }
        if(nextBall = 2){
            ballColor = "Blue";
        }
        telemetry.addData(">", "Looking for " + ballColor + " ball");
        telemetry.addData("(DPAD RIGHT) Ball Bumper 1?", toggleMap1.dpad_right);
        telemetry.addData("(DPAD DOWN) Ball Bumper 2?", toggleMap1.dpad_down);
        telemetry.addData("(DPAD LEFT) Ball Bumper 3?", toggleMap1.dpad_left);
    }
    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }
}
