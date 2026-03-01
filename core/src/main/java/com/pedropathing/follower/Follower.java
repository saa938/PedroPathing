package com.pedropathing.follower;

import com.pedropathing.ErrorCalculator;
import com.pedropathing.VectorCalculator;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathPoint;
import com.pedropathing.util.PoseHistory;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.callbacks.PathCallback;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

import java.util.ArrayDeque;
import java.util.Queue;

public class Follower {
    public FollowerConstants constants;
    public PathConstraints pathConstraints;
    public PoseTracker poseTracker;
    public ErrorCalculator errorCalculator;
    public VectorCalculator vectorCalculator;
    public Drivetrain drivetrain;
    private final PoseHistory poseHistory;
    private Pose currentPose = new Pose();
    private PathPoint closestPose = new PathPoint();
    private PathPoint previousClosestPose = new PathPoint();
    private Path currentPath = null;
    private PathChain currentPathChain = null;

    private int BEZIER_CURVE_SEARCH_LIMIT;
    private int chainIndex;
    private boolean followingPathChain, holdingPosition, isBusy, isTurning, reachedParametricPathEnd, holdPositionAtEnd, manualDrive;
    private boolean automaticHoldEnd, useHoldScaling = true;
    private double globalMaxPower = 1, centripetalScaling;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;
    private double turnHeadingErrorThreshold;
    private long reachedParametricPathEndTime;
    public boolean useTranslational = true;
    public boolean useCentripetal = true;
    public boolean useHeading = true;
    public boolean useDrive = true;
    public boolean usePredictiveBraking = true;
    private Timer zeroVelocityDetectedTimer = null;
    private Runnable resetFollowing = null;
    private Queue<PathCallback> currentCallbacks;

    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain, PathConstraints pathConstraints) {
        this.constants = constants;
        this.pathConstraints = pathConstraints;

        poseTracker = new PoseTracker(localizer);
        errorCalculator = new ErrorCalculator(constants);
        vectorCalculator = new VectorCalculator(constants);
        this.drivetrain = drivetrain;
        poseHistory = new PoseHistory(poseTracker);

        BEZIER_CURVE_SEARCH_LIMIT = constants.BEZIER_CURVE_SEARCH_LIMIT;
        holdPointTranslationalScaling = constants.holdPointTranslationalScaling;
        holdPointHeadingScaling = constants.holdPointHeadingScaling;
        centripetalScaling = constants.centripetalScaling;
        turnHeadingErrorThreshold = constants.turnHeadingErrorThreshold;
        automaticHoldEnd = constants.automaticHoldEnd;
        usePredictiveBraking = constants.usePredictiveBraking;

        breakFollowing();
    }

    public void updateConstants() {
        this.BEZIER_CURVE_SEARCH_LIMIT = constants.BEZIER_CURVE_SEARCH_LIMIT;
        this.holdPointTranslationalScaling = constants.holdPointTranslationalScaling;
        this.holdPointHeadingScaling = constants.holdPointHeadingScaling;
        this.centripetalScaling = constants.centripetalScaling;
        this.turnHeadingErrorThreshold = constants.turnHeadingErrorThreshold;
        this.automaticHoldEnd = constants.automaticHoldEnd;
        this.usePredictiveBraking = !manualDrive && constants.usePredictiveBraking;
    }

    public Follower(FollowerConstants constants, Localizer localizer, Drivetrain drivetrain) {
        this(constants, localizer, drivetrain, PathConstraints.defaultConstraints);
    }

    public void setCentripetalScaling(double set) {
        centripetalScaling = set;
    }

    public void setMaxPower(double set) {
        globalMaxPower = set;
        drivetrain.setMaxPowerScaling(set);
    }

    public Pose getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }

    public void setPose(Pose pose) {
        poseTracker.setPose(pose);
    }

    public void setX(double x) {
        poseTracker.getLocalizer().setX(x);
    }

    public void setY(double y) {
        poseTracker.getLocalizer().setY(y);
    }

    public void setHeading(double heading) {
        poseTracker.getLocalizer().setHeading(heading);
    }

    public Pose getPose() {
        return poseTracker.getPose();
    }

    public Vector getVelocity() {
        return poseTracker.getVelocity();
    }

    public void setStartingPose(Pose pose) {
        poseTracker.setStartingPose(pose);
    }

    public void holdPoint(BezierPoint point, double heading, boolean useHoldScaling) {
        breakFollowing();
        holdingPosition = true;
        this.useHoldScaling = useHoldScaling;
        isBusy = false;
        followingPathChain = false;
        setPath(new Path(point));
        currentPath.setConstantHeadingInterpolation(heading);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
    }

    public void holdPoint(BezierPoint point, double heading) {
        holdPoint(point, heading, true);
    }

    public void holdPoint(Pose pose) {
        holdPoint(new BezierPoint(pose), pose.getHeading());
    }

    public void holdPoint(Pose pose, boolean useHoldScaling) {
        holdPoint(new BezierPoint(pose), pose.getHeading(), useHoldScaling);
    }

    public void followPath(Path path, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(globalMaxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        setPath(path);
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
    }

    public void followPath(Path path) {
        followPath(path, automaticHoldEnd);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        followPath(pathChain, globalMaxPower, holdEnd);
    }

    public void followPath(PathChain pathChain) {
        followPath(pathChain, automaticHoldEnd);
    }

    public void followPath(PathChain pathChain, double maxPower, boolean holdEnd) {
        drivetrain.setMaxPowerScaling(maxPower);
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        setPath(pathChain.getPath(chainIndex));
        previousClosestPose = closestPose;
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        currentPathChain.resetCallbacks();
        currentCallbacks = currentPathChain.getNextPathCallbacks(chainIndex);

        for (PathCallback callback : currentCallbacks) {
            callback.initialize();
        }
    }

    public void resumePathFollowing() {
        if (resetFollowing != null) {
            resetFollowing.run();
            resetFollowing = null;
            breakFollowing();
            isBusy = true;
            previousClosestPose = closestPose;
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        }
    }

    public void pausePathFollowing() {
        isBusy = false;

        boolean prevHoldEnd = holdPositionAtEnd;

        if (followingPathChain && currentPathChain != null) {
            PathChain lastChain = currentPathChain;
            int lastIndex = chainIndex;

            resetFollowing = () -> {
                followingPathChain = true;
                chainIndex = lastIndex;
                currentPathChain = lastChain;
                holdPositionAtEnd = prevHoldEnd;
                currentPath = currentPathChain.getPath(lastIndex);
            };
        } else if (currentPath != null) {
            Path lastPath = currentPath;

            resetFollowing = () -> {
                holdPositionAtEnd = prevHoldEnd;
                currentPath = lastPath;
            };
        }

        holdPoint(getPose());
    }

    public void startTeleopDrive() {
        breakFollowing();
        manualDrive = true;
        update();
        drivetrain.startTeleopDrive();
    }

    public void startTeleopDrive(boolean useBrakeMode) {
        breakFollowing();
        manualDrive = true;
        update();
        drivetrain.startTeleopDrive(useBrakeMode);
    }

    public void startTeleOpDrive(boolean useBrakeMode) {
        startTeleopDrive(useBrakeMode);
    }

    public void startTeleOpDrive() {
        startTeleopDrive();
    }

    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric, double offsetHeading) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric, offsetHeading);
    }

    public void setTeleOpDrive(double forward, double strafe, double turn, double offsetHeading) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, true, offsetHeading);
    }

    public void setTeleOpDrive(double forward, double strafe, double turn, boolean isRobotCentric) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn, isRobotCentric);
    }

    public void setTeleOpDrive(double forward, double strafe, double turn) {
        vectorCalculator.setTeleOpMovementVectors(forward, strafe, turn);
    }

    public void updateDrivetrain() {
        drivetrain.updateConstants();
    }

    public void updatePose() {
        poseTracker.update();
        currentPose = poseTracker.getPose();
        poseHistory.update();
    }

    public void updateErrors() {
        errorCalculator.update(currentPose, currentPath, currentPathChain, followingPathChain, closestPose.getPose(), poseTracker.getVelocity(), chainIndex, drivetrain.xVelocity(), drivetrain.yVelocity(), getClosestPointHeadingGoal(), usePredictiveBraking);
    }

    public void updateVectors() {
        vectorCalculator.update(useDrive, useHeading, useTranslational, useCentripetal,
                manualDrive, chainIndex,
                drivetrain.getMaxPowerScaling(), followingPathChain,
                centripetalScaling, currentPose, closestPose.getPose(),
                poseTracker.getVelocity(), currentPath,
                currentPathChain, useDrive && !holdingPosition ?
                        getDriveError() : -1, getTranslationalError(),
                getHeadingError(), getClosestPointHeadingGoal(),
                getTotalDistanceRemaining(), usePredictiveBraking);
    }

    public void updateErrorAndVectors() { updateErrors(); updateVectors(); }

    private double allocatePower(double requested, double budget) {
        return Math.copySign(
                Math.min(Math.abs(requested), budget),
                requested
        );
    }

    public void update() {
        poseHistory.update();
        updateConstants();
        updatePose();
        updateDrivetrain();

        if (manualDrive) {
            previousClosestPose = closestPose;
            closestPose = new PathPoint();
            updateErrorAndVectors();

            double turnPower = getTeleopHeadingVector().dot(new Vector(1.0, currentPose.getHeading()));

            Vector teleopDrive = getTeleopDriveVector().copy();
            teleopDrive.rotateVector(-currentPose.getHeading());
            teleopDrive.setOrthogonalComponents(teleopDrive.getXComponent(), -teleopDrive.getYComponent());

            drivetrain.followVector(teleopDrive, turnPower, new Vector());

            return;
        }

        if (currentPath == null) {
            return;
        }

        if (holdingPosition) {
            previousClosestPose = closestPose;
            if (followingPathChain) currentPathChain.update();
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), 1);
            updateErrorAndVectors();
            drivetrain.runDrive(
                    useHoldScaling ? getTranslationalCorrection().times(holdPointTranslationalScaling) : getTranslationalCorrection(),
                    useHoldScaling ? getHeadingVector().times(holdPointHeadingScaling) : getHeadingVector(),
                    new Vector(),
                    poseTracker.getPose().getHeading()
            );

            if (Math.abs(getHeadingError()) < turnHeadingErrorThreshold && isTurning) {
                isTurning = false;
                isBusy = false;
            }
            return;
        }

        if (isBusy) {
            previousClosestPose = closestPose;
            if (followingPathChain) currentPathChain.update();
            closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
            updateErrorAndVectors();
            if (followingPathChain) updateCallbacks();

            Vector tangent = currentPath.getClosestPointTangentVector().normalize();
            Vector normal  = currentPath.getClosestLeftGradientVector().normalize();

            double normalPower  = getCorrectiveVector().dot(normal);
            double tangentPower = getDriveVector().dot(tangent);
            double headingPower = getHeadingVector().dot(new Vector(1.0, currentPose.getHeading()));

            // PRIORITIZE NORMAL -> HEADING -> TANGENT
            // Translational correction (normal) should get first shot at the budget so we don't starve lateral corrections.
            double normalUsed  = allocatePower(normalPower, globalMaxPower);
            double rem1 = Math.sqrt(Math.max(0.0, globalMaxPower * globalMaxPower - normalUsed * normalUsed));

            double headingUsed = allocatePower(headingPower, rem1);
            double rem2 = Math.sqrt(Math.max(0.0, rem1 * rem1 - headingUsed * headingUsed));

            double tangentUsed = allocatePower(tangentPower, rem2);

            // Small deadband: avoid tiny opposing tangential commands that provoke clampReversePower.
            // This prevents slow drift caused by aggressive small reverse-clamps.
            final double TANGENT_DEADBAND = 0.12;
            if (Math.abs(tangentUsed) < TANGENT_DEADBAND) {
                tangentUsed = 0.0;
            }

            // If we've reached the parametric end, stop applying tangential corrections entirely
            // so the robot can settle using translational/heading controllers only.
            if (reachedParametricPathEnd || (currentPath != null && currentPath.getClosestPointTValue() > 0.98)) {
                tangentUsed = 0.0;
            }

            Vector fieldDrivePower = normal.times(normalUsed).plus(tangent.times(tangentUsed));

            Vector robotDrivePower = fieldDrivePower.copy();
            robotDrivePower.rotateVector(-currentPose.getHeading());

            Vector robotVelocity = poseTracker.getVelocity().copy();
            robotVelocity.rotateVector(-currentPose.getHeading());

            drivetrain.followVector(robotDrivePower, headingUsed, robotVelocity);
        }

        if (poseTracker.getVelocity().getMagnitude() < 1.0 && currentPath.getClosestPointTValue() > 0.8
                && zeroVelocityDetectedTimer == null && isBusy) {
            zeroVelocityDetectedTimer = new Timer();
        }

        double tangentDot = getDriveVector().dot(getClosestPointTangentVector().normalize());
        boolean nextPathWithinBrakingDistance =
                followingPathChain
                        && chainIndex < currentPathChain.size() - 1
                        && usePredictiveBraking
                        && currentPath.getClosestPointTValue() > 0.95       // tighter threshold
                        && tangentDot > 0.05                                 // require meaningful forward component
                        && tangentDot < globalMaxPower * 0.6;                // stricter bound so we really are braking

        if (!(currentPath.isAtParametricEnd()
                || nextPathWithinBrakingDistance
                || (zeroVelocityDetectedTimer != null
                && zeroVelocityDetectedTimer.getElapsedTime() > 500.0))) {
            return;
        }

        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            advanceToNextPath();
            return;
        }

        if (!reachedParametricPathEnd) {
            reachedParametricPathEnd = true;
            reachedParametricPathEndTime = System.currentTimeMillis();
        }

        updateErrorAndVectors();
        if (!(
                (
                        System.currentTimeMillis() - reachedParametricPathEndTime
                                > currentPath.getPathEndTimeoutConstraint()
                )
                        || (
                        poseTracker.getVelocity().getMagnitude()
                                < currentPath.getPathEndVelocityConstraint()
                )
                        && (
                        poseTracker.getPose().distanceFrom(closestPose.getPose())
                                < currentPath.getPathEndTranslationalConstraint()
                )
                        && (
                        MathFunctions.getSmallestAngleDifference(poseTracker.getPose().getHeading(), getClosestPointHeadingGoal())
                                < currentPath.getPathEndHeadingConstraint()
                )
        )) {
            return;
        }

        if (holdPositionAtEnd) {
            holdPositionAtEnd = false;
            if (followingPathChain) holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPathChain.getHeadingGoal(new PathChain.PathT(currentPathChain.size() - 1, 1)));
            else holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
        } else {
            breakFollowing();
        }
    }

    private void advanceToNextPath() {
        breakFollowing();
        isBusy = true;
        followingPathChain = true;
        chainIndex++;
        setPath(currentPathChain.getPath(chainIndex));
        previousClosestPose = closestPose;
        if (followingPathChain) currentPathChain.update();
        closestPose = currentPath.updateClosestPose(poseTracker.getPose(), BEZIER_CURVE_SEARCH_LIMIT);
        updateErrorAndVectors();
        currentCallbacks = currentPathChain.getNextPathCallbacks(chainIndex);

        for (PathCallback callback : currentCallbacks) {
            callback.initialize();
        }
    }

    public void updateCallbacks() {
        for (PathCallback callback : currentCallbacks) {
            if (callback.isReady()) {
                callback.run();
            }
        }
    }

    public void breakFollowing() {
        errorCalculator.breakFollowing();
        vectorCalculator.breakFollowing();
        drivetrain.breakFollowing();
        manualDrive = false;
        holdingPosition = false;
        isBusy = false;
        isTurning = false;
        reachedParametricPathEnd = false;
        zeroVelocityDetectedTimer = null;
    }

    public boolean isBusy() { return isBusy; }
    public PathPoint getClosestPose() { return closestPose; }

    public boolean atParametricEnd() {
        if (currentPath == null){ return true; }
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    public PathBuilder pathBuilder(PathConstraints constraints) { return new PathBuilder(this, constraints); }
    public PathBuilder pathBuilder() { return new PathBuilder(this); }
    public double getTotalHeading() { return poseTracker.getTotalHeading(); }
    public Path getCurrentPath() { return currentPath; }
    public boolean isRobotStuck() { return zeroVelocityDetectedTimer != null; }
    public boolean isLocalizationNAN() { return poseTracker.getLocalizer().isNAN(); }

    @Deprecated
    public void turn(double radians, boolean isLeft) { turn(isLeft ? radians : -radians); }

    public void turn(double radians) {
        Pose temp = new Pose(getPose().getX(), getPose().getY(), getPose().getHeading() + radians);
        holdPoint(temp);
        isTurning = true;
        isBusy = true;
    }

    public void turnTo(double radians) {
        holdPoint(new Pose(getPose().getX(), getPose().getY(), radians));
        isTurning = true;
        isBusy = true;
    }

    @Deprecated
    public void turnToDegrees(double degrees) { turnTo(Math.toRadians(degrees)); }

    @Deprecated
    public void turnDegrees(double degrees, boolean isLeft) { turn(Math.toRadians(degrees), isLeft); }

    public boolean isTurning() { return isTurning; }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance, double headingTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance && Math.abs(pose.getHeading() - getPose().getHeading()) < headingTolerance;
    }

    public boolean atPose(Pose pose, double xTolerance, double yTolerance) {
        return Math.abs(pose.getX() - getPose().getX()) < xTolerance && Math.abs(pose.getY() - getPose().getY()) < yTolerance;
    }

    public void setMaxPowerScaling(double maxPowerScaling) { drivetrain.setMaxPowerScaling(maxPowerScaling); }
    public double getMaxPowerScaling() { return drivetrain.getMaxPowerScaling(); }
    public boolean getUseDrive() { return useDrive; }
    public boolean getUseHeading() { return useHeading; }
    public boolean getUseTranslational() { return useTranslational; }
    public boolean getUseCentripetal() { return useCentripetal; }
    public boolean getTeleopDrive() { return manualDrive; }
    public int getChainIndex() { return chainIndex; }
    public PathChain getCurrentPathChain() { return currentPathChain; }
    public boolean getFollowingPathChain() { return followingPathChain; }
    public double getCentripetalScaling() { return centripetalScaling; }
    public boolean isTeleopDrive() { return manualDrive; }

    public Vector getTeleopHeadingVector() { return vectorCalculator.getTeleopHeadingVector(); }
    public Vector getTeleopDriveVector() { return vectorCalculator.getTeleopDriveVector(); }
    public double getHeadingError() { return errorCalculator.getHeadingError(); }
    public Vector getTranslationalError() { return errorCalculator.getTranslationalError(); }
    public double getDriveError() { return errorCalculator.getDriveError(); }
    public Vector getDriveVector() { return vectorCalculator.getDriveVector(); }
    public Vector getCorrectiveVector() { return vectorCalculator.getCorrectiveVector(); }
    public Vector getHeadingVector() { return vectorCalculator.getHeadingVector(); }
    public Vector getTranslationalCorrection() { return vectorCalculator.getTranslationalCorrection(); }
    public Vector getCentripetalForceCorrection() { return vectorCalculator.getCentripetalForceCorrection(); }
    public PathConstraints getConstraints() { return pathConstraints; }
    public FollowerConstants getConstants() { return constants; }
    public void setConstraints(PathConstraints pathConstraints) { this.pathConstraints = pathConstraints; }
    public Drivetrain getDrivetrain() { return drivetrain; }
    public PoseTracker getPoseTracker() { return poseTracker; }
    public ErrorCalculator getErrorCalculator() { return errorCalculator; }
    public VectorCalculator getVectorCalculator() { return vectorCalculator; }
    public PoseHistory getPoseHistory() { return poseHistory; }
    public void setXVelocity(double vel) { drivetrain.setXVelocity(vel); }
    public void setYVelocity(double vel) { drivetrain.setYVelocity(vel); }
    public void setDrivePIDFCoefficients(FilteredPIDFCoefficients drivePIDFCoefficients) { vectorCalculator.setDrivePIDFCoefficients(drivePIDFCoefficients); }
    public void setSecondaryDrivePIDFCoefficients(FilteredPIDFCoefficients secondaryDrivePIDFCoefficients) { vectorCalculator.setSecondaryDrivePIDFCoefficients(secondaryDrivePIDFCoefficients); }
    public void setHeadingPIDFCoefficients(PIDFCoefficients headingPIDFCoefficients) { vectorCalculator.setHeadingPIDFCoefficients(headingPIDFCoefficients); }
    public void setSecondaryHeadingPIDFCoefficients(PIDFCoefficients secondaryHeadingPIDFCoefficients) { vectorCalculator.setSecondaryHeadingPIDFCoefficients(secondaryHeadingPIDFCoefficients); }
    public void setTranslationalPIDFCoefficients(PIDFCoefficients translationalPIDFCoefficients) { vectorCalculator.setTranslationalPIDFCoefficients(translationalPIDFCoefficients); }
    public void setSecondaryTranslationalPIDFCoefficients(PIDFCoefficients secondaryTranslationalPIDFCoefficients) { vectorCalculator.setSecondaryTranslationalPIDFCoefficients(secondaryTranslationalPIDFCoefficients); }

    public void setConstants(FollowerConstants constants) {
        this.constants = constants;
        updateConstants();
        errorCalculator.setConstants(constants);
        vectorCalculator.setConstants(constants);
        drivetrain.updateConstants();
    }

    public double getHeadingGoal(double t) {
        if (currentPathChain != null) {
            return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, t));
        }
        return currentPath.getHeadingGoal(t);
    }

    private double getHeadingGoal(PathPoint point) {
        if (currentPath == null) return 0;
        if (currentPathChain != null) return currentPathChain.getHeadingGoal(new PathChain.PathT(chainIndex, point.tValue));
        return currentPath.getHeadingGoal(point);
    }

    public double getClosestPointHeadingGoal() {
        if (currentPath == null) return 0;
        if (followingPathChain && currentPathChain != null)
            return currentPathChain.getClosestPointHeadingGoal(new PathChain.PathT(chainIndex, closestPose.tValue));
        return currentPath.getHeadingGoal(closestPose);
    }

    public Vector getClosestPointTangentVector() {
        return getClosestPose().getTangentVector();
    }

    public void activateAllPIDFs() {
        useDrive = true;
        useHeading = true;
        useTranslational = true;
        useCentripetal = true;
    }

    public void deactivateAllPIDFs() {
        useDrive = false;
        useHeading = false;
        useTranslational = false;
        useCentripetal = false;
    }

    public void activateDrive() { useDrive = true; }
    public void activateHeading() { useHeading = true; }
    public void activateTranslational() { useTranslational = true; }
    public void activateCentripetal() { useCentripetal = true; }

    public double getDistanceTraveledOnPath() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.getDistanceTraveled();
    }

    public double getPathCompletion() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.getPathCompletion();
    }

    public double getDistanceRemaining() {
        if (currentPath == null) {
            return 0;
        }
        return currentPath.getDistanceRemaining();
    }

    public String[] debug() {
        String[] info = new String[4];
        info[0] = poseTracker.debugString();
        info[1] = errorCalculator.debugString();
        info[2] = vectorCalculator.debugString();
        info[3] = drivetrain.debugString();
        return info;
    }

    public Vector getAcceleration() { return poseTracker.getAcceleration(); }
    public double getAngularVelocity() { return poseTracker.getAngularVelocity(); }
    private void setPath(Path path) { this.currentPath = path; currentPath.init(); }
    public PathPoint getPreviousClosestPose() { return previousClosestPose; }
    public double getTangentialVelocity() { return getVelocity().dot(getClosestPointTangentVector().normalize()); }
    public double getHeading() { return getPose().getHeading(); }

    public double getTotalDistanceRemaining() {
        if (currentPath == null) { return 0; }
        if (!followingPathChain) { return currentPath.getDistanceRemaining(); }
        PathChain.DecelerationType type = currentPathChain.getDecelerationType();
        if (type == PathChain.DecelerationType.NONE) { return -1; }
        return currentPathChain.getDistanceRemaining(chainIndex);
    }
}