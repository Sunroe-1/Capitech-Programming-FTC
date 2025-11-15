    /**
     * The stop method is called when the OpMode is stopped.
     */
    @Override
    public void stop() {
        try {
            // Set all drive motors to zero power to ensure the robot stops cleanly.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        } catch (Exception e) {
            telemetry.addData("Error", "Exception occurred while stopping: " + e.getMessage());
            telemetry.update();
        }
