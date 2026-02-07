package com.pedropathing.drivetrain;

import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;

public abstract class Drivetrain {
    /**
     * These are the movement vectors for the drivetrain, essentially the directions of the forces created by each wheel, such that x-components are scaled by the x velocity and y-components are scaled by the y velocity.
     */
    protected Vector[] vectors;

    /**
     * This is the maximum power scaling for the drivetrain. This is used to limit the maximum
     * power that can be applied to the motors, which is useful for preventing damage to the
     * drivetrain or for controlling the speed of the robot.
     */
    protected double maxPowerScaling;

    /**
     * This is used to determine whether the drivetrain should use voltage compensation or not.
     */
    protected boolean voltageCompensation;

    /**
     * This is the nominal voltage for the drivetrain. This is used for voltage compensation.
     * It is set to 12.0V by default, which is the nominal voltage for most FTC robots.
     */
    protected double nominalVoltage;

    /**
     * Maximum absolute braking power applied when commanded power is opposite of motion.
     * Default 0.2; can be tuned by callers via setter.
     */
    protected double maxReverseBrakingPower = 0.2;

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array.
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public abstract double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    /**
     * This sets the maximum power scaling for the drivetrain. This is used to limit the maximum
     * power that can be applied to the motors, which is useful for preventing damage to the
     * drivetrain or for controlling the speed of the robot.
     *
     * @param maxPowerScaling this is a double between 0 and 1 inclusive that represents the maximum
     *                        power scaling factor.
     */
    public void setMaxPowerScaling(double maxPowerScaling) {
        this.maxPowerScaling = MathFunctions.clamp(maxPowerScaling, 0, 1);
    }

    /**
     * This gets the maximum power scaling for the drivetrain. This is used to limit the maximum
     * power that can be applied to the motors.
     *
     * @return this returns a double between 0 and 1 inclusive that represents the maximum power
     *         scaling factor.
     */
    public double getMaxPowerScaling() {
        return maxPowerScaling;
    }

    /**
     * This updates the constants used by the drivetrain.
     */
    public abstract void updateConstants();

    /**
     * This is used to break the drivetrain's following. This is useful for stopping the robot from following a Path or PathChain.
     */
    public abstract void breakFollowing();

    /**
     * This runs the drivetrain with the specified drive powers. This is used to set the power of the motors directly.
     *
     * @param drivePowers this is an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public abstract void runDrive(double[] drivePowers);

    /**
     * This gets the drive powers and runs them immediately.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     */
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    /**
     * Convenience: follow a field-relative translation vector and a turn power scalar.
     *
     * This method:
     *  1. Converts the supplied **field** vector into the robot frame (so X/Y align to robot axes).
     *  2. Uses drivetrain's robot-relative velocities (xVelocity/yVelocity) to detect opposing motion.
     *  3. Clamps per-axis commanded power if it opposes the current motion, using maxReverseBrakingPower.
     *  4. Converts the clamped robot-relative vector back to the field frame.
     *  5. Builds a heading Vector from the scalar turnPower and calls runDrive(...) so concrete drivetrains handle wheel mapping.
     *
     * @param fieldVector field-relative translational drive vector (direction + magnitude)
     * @param turnPower scalar rotational command (positive -> turn one way, negative the other)
     * @param robotHeading robot's current heading (radians)
     */
    public void followFieldVector(Vector fieldVector, double turnPower, double robotHeading) {
        // 1) to robot frame
        Vector robotCmd = fieldToRobot(fieldVector, robotHeading);

        // 2) robot-relative velocity (abstract accessors implemented by concrete drivetrain)
        double rvx = xVelocity();
        double rvy = yVelocity();

        // 3) clamp opposing motion per-axis
        double cx = clampReversePower(robotCmd.getXComponent(), rvx);
        double cy = clampReversePower(robotCmd.getYComponent(), rvy);

        // 4) back to field frame
        Vector clampedField = robotToField(new Vector(cx, cy), robotHeading);

        // 5) turn scalar -> heading Vector (heading vector points in direction of robot heading)
        Vector headingVec = new Vector(Math.cos(robotHeading), Math.sin(robotHeading)).times(turnPower);

        // reuse existing pipeline
        runDrive(clampedField, headingVec, new Vector(), robotHeading);
    }

    /**
     * Convert a field-relative vector to robot-relative using heading (radians).
     */
    protected Vector fieldToRobot(Vector field, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double fx = field.getXComponent();
        double fy = field.getYComponent();

        double rx = cos * fx + sin * fy;
        double ry = -sin * fx + cos * fy;

        return new Vector(rx, ry);
    }

    /**
     * Convert a robot-relative vector back to field frame.
     */
    protected Vector robotToField(Vector robot, double heading) {
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double rx = robot.getXComponent();
        double ry = robot.getYComponent();

        double fx = cos * rx - sin * ry;
        double fy = sin * rx + cos * ry;

        return new Vector(fx, fy);
    }

    /**
     * Clamp opposing power per-axis relative to directionOfMotion (robot-relative velocity component).
     * If the commanded power is in the opposite direction of motion, limit its magnitude to maxReverseBrakingPower.
     */
    protected double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        if (!isOpposingMotion) return power;

        if (power < 0) return Math.max(power, -Math.abs(maxReverseBrakingPower));
        else return Math.min(power, Math.abs(maxReverseBrakingPower));
    }

    /**
     * Setter/getter for the brake clamp magnitude so teams can tune or externalize it to coefficients.
     */
    public void setMaxReverseBrakingPower(double maxReverseBrakingPower) {
        this.maxReverseBrakingPower = Math.abs(maxReverseBrakingPower);
    }

    public double getMaxReverseBrakingPower() {
        return this.maxReverseBrakingPower;
    }

    /**
     * This starts the TeleOp drive mode. This is used to set the drivetrain into TeleOp mode, where
     * it can be controlled by the driver.
     */
    public abstract void startTeleopDrive();

    /**
     * This starts the TeleOp drive mode with a specified brake mode. This is used to set the drivetrain
     * into TeleOp mode, where it can be controlled by the driver, and allows for setting the brake mode.
     *
     * @param brakeMode this is a boolean that specifies whether the drivetrain should use brake mode or not.
     */
    public abstract void startTeleopDrive(boolean brakeMode);

    /**
     * This gets the current x velocity of the drivetrain.
     * @return this returns the x velocity of the drivetrain.
     */
    public abstract double xVelocity();

    /**
     * This gets the current y velocity of the drivetrain.
     * @return this returns the y velocity of the drivetrain.
     */
    public abstract double yVelocity();

    /**
     * This sets the x velocity of the drivetrain.
     * @param xMovement this is the x velocity to set.
     */
    public abstract void setXVelocity(double xMovement);

    /**
     * This sets the y velocity of the drivetrain.
     * @param yMovement this is the y velocity to set.
     */
    public abstract void setYVelocity(double yMovement);

    /**
     * This sets whether the drivetrain should use voltage compensation or not.
     * @param use this is a boolean that specifies whether the drivetrain should use voltage compensation or not.
     */
    public void useVoltageCompensation(boolean use) {
        this.voltageCompensation = use;
    }

    /**
     * This gets whether the drivetrain is using voltage compensation or not.
     * @return this returns a boolean that specifies whether the drivetrain is using voltage compensation or not.
     */
    public boolean isVoltageCompensation() {
        return voltageCompensation;
    }

    /**
     * This gets the nominal voltage for the drivetrain.
     * @return this returns the nominal voltage for the drivetrain.
     */
    public double getNominalVoltage() {
        return nominalVoltage;
    }

    /**
     * This sets the nominal voltage for the drivetrain. This is used for voltage compensation.
     * @param set this is the nominal voltage to set.
     */
    public void setNominalVoltage(double set) {
        this.nominalVoltage = set;
    }

    /**
     * This is used to get the voltage of the drivetrain. This is useful for debugging purposes.
     * It should be called periodically to get the current voltage of the drivetrain.
     */
    public abstract double getVoltage();

    /**
     * This is used to get a debug string for the drivetrain. This is useful for debugging purposes.
     *
     * @return this returns a String that contains the debug information for the drivetrain.
     */
    public abstract String debugString();

}
