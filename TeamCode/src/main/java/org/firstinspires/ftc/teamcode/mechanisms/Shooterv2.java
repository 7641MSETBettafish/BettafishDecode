package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooterv2 {

    //left and right shooter motor PID weights
    public static double P = 0.001;
    public static double I = 0;
    public static double D = 0.05;

    //FF weight


    public static double kF = 0.05;

    //constant used in smoothing RPM. lower value more smooth but less responsive. higher value less smooth but more responsive
    public static double RPMAlpha = 0.05;

    //so that you can turn debugging on and off in ftc dashboard
    public static double RPM_JITTER = 30;
    public static boolean debug = true;
    public static boolean debugEMA = true;

    // TODO: enter common field distances in inches (for auto)
    //common field distances
    public static double Close = 0;
    public static double Middle = 0;
    public static double Far = 0;

    public static double bangTolerance = 150;
    public static double bangPower = 0.05;

    private static final double GRAVITY = 386.09; // in/s² (imperial gravity)
    private static final double LAUNCH_HEIGHT = 13.5; // inches
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(54.5);

    // ---- Goal ----
    private static final double GOAL_FRONT_HEIGHT = 39.0; // inches
    private static final double GOAL_BACK_HEIGHT = 54.0;  // inches
    private static final double GOAL_DEPTH = 18.0;        // inches

    // Aim for middle of the goal
    private static final double TARGET_DEPTH = GOAL_DEPTH / 2.0;
    private static final double TARGET_HEIGHT = GOAL_FRONT_HEIGHT + (GOAL_BACK_HEIGHT - GOAL_FRONT_HEIGHT) * (TARGET_DEPTH / GOAL_DEPTH);

    // ---- Ball ----
    private static final double BALL_MASS = 0.156 * 0.45359237; // lb to kg

    // ---- Flywheels ----
    private static final double FLYWHEEL_RADIUS = 0.072 / 2.0; //m
    private static final double FLYWHEEL_MASS = 0.056; // kg

    // ---- Motors ----
    public static final double motorTicksPerRevolution = 103.8;
    public static final double MOTOR_NO_LOAD_RPM = 1300; //1620.0
    public static final double GEAR_RATIO = 2.5; // 1:2.5 gearing up



    // ---- Conversion ----
    private static final double INCH_TO_METER = 0.0254;

    //distances enum for the common field distances
    public enum Distances {
        //set each state to the correct field distances
        CLOSE(Close),
        MIDDLE(Middle),
        FAR(Far);

        private final double d;

        //constructor for each state
        Distances(double d) {
            this.d = d;
        }

        //method to get the distance from the enum
        private double dis() {
            return d;
        }
    }

    public DcMotor leftShooterMotor;
    public DcMotor rightShooterMotor;

    public double lastLeftPosition;
    public double lastRightPosition;

    public double leftRPM;
    public double rightRPM;
    public boolean RPMInit;

    public ElapsedTime RPMTimer;

    public PIDController pid;

    public double targetRPM = 0;
    public double lastTarget;

    //shooter constructor
    public Shooterv2(HardwareMap HWMap) {
        //initialize motors
        leftShooterMotor = HWMap.get(DcMotor.class, "leftShooter");
        rightShooterMotor = HWMap.get(DcMotor.class, "rightShooter");

        //reset motor encoder values
        leftShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //turn motors back on after resetting encoders
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //make sure both spin in the same direction
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //initialize instance variables
        RPMTimer = new ElapsedTime();
        RPMTimer.reset();
        lastLeftPosition = 0;
        lastRightPosition = 0;
        RPMInit = false;

        pid = new PIDController(P, I, D);

    }

    public void updateBang() {
        double avgRPM = (leftRPM + rightRPM) / 2.0;
        double error = targetRPM - avgRPM;
        double power = leftShooterMotor.getPower();

        if (error > bangTolerance) {

            power += bangPower;
        } else if (error < -bangTolerance) {
            power -= bangPower;
        }


        power = Math.max(0.0, Math.min(1.0, power));

        setPower(power);
    }


    public void verySmoothRPM(double currentLeftRPM, double currentRightRPM, double currentTime) {
        if (!RPMInit) {
            leftRPM = currentLeftRPM;
            rightRPM = currentRightRPM;
            RPMInit = true;
        } else {
            double effectiveAlpha = Math.min(1.0, Math.max(0.0, RPMAlpha * (currentTime / 10)));
            //this is jitter REJECTION
            double leftDelta = currentLeftRPM - leftRPM;
            double rightDelta = currentRightRPM - rightRPM;

            if (Math.abs(leftDelta) > RPM_JITTER) {
                leftRPM += effectiveAlpha * leftDelta;

            }

            if (Math.abs(rightDelta) > RPM_JITTER) {
                rightRPM += effectiveAlpha * rightDelta;
            }
        }
    }


    //calculates RPM based on function (current not working)
    public static double calculateRPM(double distance) {

        // Target horizontal distance
        double depthSigmoid = TARGET_DEPTH / (1.0 + Math.exp(0.4 * (distance - 50)));
        double x = distance + depthSigmoid;

        // Target vertical distance
        double y = TARGET_HEIGHT - LAUNCH_HEIGHT;

        // required launch speed
        // y = x*tanθ - g*x²/(2*v²*cos²θ)
        double theta = LAUNCH_ANGLE_RAD;
        double tanTheta = Math.tan(theta);
        double cosTheta = Math.cos(theta);

        double numerator = GRAVITY * x * x;
        double denominator = 2.0 * cosTheta * cosTheta * (x * tanTheta - y);
        double vRequired_in_per_s = Math.sqrt(numerator / denominator);

        // convert to m/s
        double vRequired_m_per_s = vRequired_in_per_s * INCH_TO_METER;

        // ball exit velocity to flywheel rim speed
        double vRim = vRequired_m_per_s * (BALL_MASS + FLYWHEEL_MASS) / FLYWHEEL_MASS;

        // rim speed to flywheel rpm
        return (vRim / (2 * Math.PI * FLYWHEEL_RADIUS)) * 60.0 / 2;

        // flywheel to motor rpm
        //flywheelRPM / GEAR_RATIO
        // convert to power fraction
        //double power = motorRPM / MOTOR_NO_LOAD_RPM;
        //if (power > 1.0) power = 1.0; // clamp

        //return power;
    }

    //checks if the both the left and right RPM are in the threshold based on the hightol (upper bound) and lowtol (lower bound)
    public static boolean RPMInThreshold(double leftRPM, double rightRPM, double targetRPM) {
        double hightol = 25;
        double lowtol = 25;
        return leftRPM > targetRPM - lowtol && leftRPM < targetRPM + hightol && rightRPM > targetRPM - lowtol && rightRPM < targetRPM + hightol;
    }

    //updates the current RPM of the shooter motors
    public void updateRPM() {
        double currentTime = RPMTimer.milliseconds();

        double currentLeftRPM = ((leftShooterMotor.getCurrentPosition() - lastLeftPosition) / motorTicksPerRevolution)
                * GEAR_RATIO * (60000 / currentTime);
        double currentRightRPM = ((rightShooterMotor.getCurrentPosition() - lastRightPosition) / motorTicksPerRevolution)
                * GEAR_RATIO * (60000 / currentTime);

        // Average both for a shared feedback
        double avgRPM = (currentLeftRPM + currentRightRPM) / 2.0;

        if (debugEMA) {
            smoothRPM(currentLeftRPM, currentRightRPM, currentTime);
        } else {
            leftRPM = avgRPM;
            rightRPM = avgRPM;
        }

        RPMTimer.reset();
        lastLeftPosition = leftShooterMotor.getCurrentPosition();
        lastRightPosition = rightShooterMotor.getCurrentPosition();

        if (debug) {
            pid = new PIDController(P, I, D);
        }
    }

    //method that takes in the current RPM and adds it to a moving average so that there are not a lot of large spikes in the RPM (better for PID)
    public void smoothRPM(double currentLeftRPM, double currentRightRPM, double currentTime) {
        //EMA for smoother values
        double currentAvg = (currentLeftRPM + currentRightRPM) / 2.0;
        if (!RPMInit) {

            RPMInit = true;
        } else {
            double effectiveAlpha = Math.min(1.0, Math.max(0.0, RPMAlpha * (currentTime / 10)));
            double smoothed = leftRPM + effectiveAlpha * (currentAvg - leftRPM);
            leftRPM = smoothed;
            rightRPM = smoothed;
        }
    }

    //method to make it easier to set the power     of both motors
    public void setPower(double p) {
        leftShooterMotor.setPower(p);
        rightShooterMotor.setPower(p);
    }

    public void updatePID() {
        //detect if the target has changed and we need to use the FF() method
        boolean targetChanged = targetRPM != lastTarget;

        double avgRPM = (leftRPM + rightRPM) / 2.0;
        //get the next PID value and add it to the current power
        //(targetChanged ? FF(leftRPM) : 0) is an if statement but with simplified syntax. Search up java ternary operator to understand
        double power = leftShooterMotor.getPower() + 0.001 * pid.calculate(avgRPM, targetRPM) + (targetChanged ? FF(avgRPM) : 0);
        power = Math.max(0.0, Math.min(1.0, power));
        leftShooterMotor.setPower(power);
        rightShooterMotor.setPower(power);


        //store target RPM into the last target RPM for the next loop
        lastTarget = targetRPM;
    }

    //A little kick when the target changes to get the inital rpm a bit closer
    public double FF(double RPM) {
        return kF * (targetRPM - RPM);
    }


    public class PowerUp implements Action {

        //distance between the robot and the goal
        double distance;
        //timer
        ElapsedTime time;
        //to keep track if it is the first time this action is being played
        boolean init = false;

        //constructor with distance given in inches
        public PowerUp(double distance) {
            //set parameter to the instance variable
            this.distance = distance;
            //initialize timer
            time = new ElapsedTime();
        }

        //constructor with distance given in the preset options (see the enum "Distances" above)
        public PowerUp(Distances distance) {
            //set distance to the enum value
            this.distance = distance.dis();
            //initialize timer
            time = new ElapsedTime();
        }

        //main method of the Action that runs until false is returned
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //check if it is the first loop
            if (!init) {
                //set the target RPM (instance variable in shooter) to the RPM calculated from the distance in calculate RPM
                targetRPM = calculateRPM(distance);
                //start timer from 0
                time.reset();
                //init true so we know that the next loops are no longer the first loop
                init = true;
            }
            updateRPM();
            updatePID();
            updateBang();



            //check if the time is over 75 milliseconds (to prevent shooting when the updateRPM() method starts up and returns a weird value)
            //check if the RPM is in threshold (see above method
            return time.milliseconds() > 75 && RPMInThreshold(leftRPM, rightRPM, targetRPM);

        }
    }

    //methods that return a new PowerUp object when you call them.
    public Action powerUp(double d) {
        return new PowerUp(d);
    }
    public Action powerUp(Distances d) {
        return new PowerUp(d);
    }


    //sets the motor powers to 0, stopping the motors. (for use in auto where you only can use actions to move the robot)
    public class Stop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //see above setPower() method explanation
            setPower(0);
            return false;
        }

    }
    public Action stop() {
        return new Shooterv2.Stop();
    }
}
