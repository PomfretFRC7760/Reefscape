# 2025 Griffin Innovations Robot Code
## Future additions

### Bugfixes
- [ ] Test/fix autonomous driving with robot rotation
- [ ] Fix gyroscope logic so it doesn't need to be zeroed at the start of teleop
### Features coming soon
- [ ] Semi autonomous control in teleop
    - [x] Robot pose from Apriltags
    - [ ] Automatic pathfinding to target pose (in progress)
    - [ ] Dynamic obstacle avoidance with neural processing
        - [x] Trained neural network detection pipeline
        - [ ] Dynamic path constraints based on obstacle detection
        - [ ] Obstacle avoidance modes for different types of obstacles, alliance color, robot location and target
    - [ ] Locate and drive to algae with neural processing (might not be added, singular limelight currently facing wrong way for this)
    - [ ] Intuitive control interface for semi autonomous (Smartdashboard sendablechoosers?)
    - [ ] Limelight pipeline switching (we only have one limelight and have to make the most of it)
    - [ ] Manual drive override (if semi auto ever craps the bed)
- [ ] PID tuning and position control for lift and intakes
- [ ] More commands for lift and intakes for autonomous
