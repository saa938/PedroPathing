package com.pedropathing.drivetrain;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.math.Vector;

/**
 * This is the CustomDrivetrain class. This is an abstract class that extends the Drivetrain class.
 * It is intended to be used as a base class for custom drivetrain implementations.
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 * @author Kabir Goyal
 */
public abstract class CustomDrivetrain extends Drivetrain {
    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;
    protected double maximumBrakingPower = 0.2; // Default clamping threshold

    /**
     * This method takes in forward, strafe, and rotation values and applies them to
     * the drivetrain.
     *
     * @param forward the forward power value, which would typically be
     *                -gamepad1.left_stick_y in a normal arcade drive setup
     * @param strafe the strafe power value, which would typically be
     *               -gamepad1.left_stick_x in a normal arcade drive setup
     *               because pedro treats left as positive
     * @param rotation the rotation power value, which would typically be
     *                 -gamepad1.right_stick_x in a normal arcade drive setup
     *                 because CCW is positive
     */
    public abstract void arcadeDrive(double forward, double strafe, double rotation);

    /**
     * Sets the maximum braking power threshold (default 0.2)
     * @param threshold the maximum power allowed when opposing velocity direction
     */
    public void setMaximumBrakingPower(double threshold) {
        this.maximumBrakingPower = Math.max(0, Math.min(1, threshold));
    }

    /**
     * Prevents the robot from applying too much power in the opposite direction of
     * the robot's momentum. Alternating full forward (+1) and full reverse (-1) power
     * causes the control hub to restart due to low voltage spikes. This prevents that by
     * capping the amount of voltage applied opposite to the direction of motion to be
     * very minimal. Even a tiny opposite voltage (e.g., -0.0001) locks the wheels like
     * zero-power brake mode, using the motor's own momentum for braking without consuming
     * significant energy.
     *
     * @param power the requested power
     * @param directionOfMotion the current velocity in that direction
     * @return the clamped power
     */
    protected double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        if (!isOpposingMotion) {
            return power;
        }
        if (power < 0) {
            return Math.max(power, -maximumBrakingPower);
        } else {
            return Math.min(power, maximumBrakingPower);
        }
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() >= maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    0
            };
        }

        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(correctivePower, headingPower, true)) {
            headingPower = scaledVector(correctivePower, headingPower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    headingPower.dot(new Vector(1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(1, robotHeading))
                };
            }
        }
    }

    protected boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    protected Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus? Math.min(findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)) :
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);
        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);
        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;

        lastTranslationalVector = translationalVector; //before rotation
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;

        translationalVector.rotateVector(-robotHeading); // field → robot frame
        arcadeDrive(translationalVector.getXComponent(), translationalVector.getYComponent(), calculatedDrive[2]);
    }

    /**
     * Follows a robot-relative vector with heading correction and per-axis reverse-power clamping.
     *
     * The robotVector arrives already in the robot frame (rotated by -heading in Follower).
     * Pedro's convention after that rotation is:
     *   X component = forward / backward direction
     *   Y component = lateral (left / right) direction
     * This matches exactly what runDrive produces before calling arcadeDrive(x, y, rot).
     *
     * We clamp forward against the forward velocity component and lateral against the lateral
     * velocity component independently. Clamping along path-tangent / path-normal directions
     * instead would spread the braking across both axes in ways that don't map cleanly to
     * mecanum wheel pairs, causing some wheels to brake while others accelerate along the same
     * maneuver.
     *
     * @param robotVector    robot-relative drive vector (X = forward, Y = lateral)
     * @param turnPower      signed turn scalar (+CCW, -CW)
     * @param robotVelocity  robot-relative velocity vector for per-axis reverse-power clamping
     */
    @Override
    public void followVector(Vector robotVector, double turnPower, Vector robotVelocity) {
        // Pedro robot frame after rotateVector(-heading): X = forward, Y = lateral.
        // Clamp each axis independently — forward braking should not bleed into lateral and vice versa.
        double forward = clampReversePower(robotVector.getXComponent(), robotVelocity.getXComponent());
        double strafe  = clampReversePower(robotVector.getYComponent(), robotVelocity.getYComponent());
        arcadeDrive(forward, strafe, turnPower);
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {}
}