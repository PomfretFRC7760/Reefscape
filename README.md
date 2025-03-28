# 2025 Griffin Innovations Robot Code
## Future additions

### Bugfixes
- [x] Test/fix autonomous driving with robot rotation
- [x] Fix gyroscope logic so it doesn't need to be zeroed at the start of teleop
- [x] Ensure gyroscope is being read correctly in auto
#### These are possibly fixed but aren't tested
### Features coming soon
- [x] Semi autonomous control in teleop
    - [x] Robot pose from Apriltags
    - [x] Automatic pathfinding to target pose
    - [ ] Dynamic obstacle avoidance with neural processing
        - [x] Trained neural network detection pipeline
        - [ ] Dynamic path constraints based on obstacle detection
        - [ ] Obstacle avoidance modes for different types of obstacles, alliance color, robot location and target
    - [x] SmartDashboard control interface to select positions
    - [x] Locate and drive to algae with neural processing (untested)
    - [x] Limelight pipeline switching (we only have one limelight and have to make the most of it)
    - [x] Manual drive override (if semi auto ever craps the bed)
- [x] PID tuning and position control for lift and intakes
- [x] More commands for lift and intakes for autonomous
- [x] Rework subsystems to account for recent hardware design changes
- [x] Better control interface (move some controls over to smartdashboard)
- [x] Build autos from smartdashboard
- [x] Option to use premade autos
- [ ] Better way of selecting positions for semi autonomous
- [ ] Algae remover subsystem and controls
