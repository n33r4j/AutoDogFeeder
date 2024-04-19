# AutoDogFeeder
An automatic dog feeder based on ROS 2(iron) running on a RPi4 B.

## Instructions to Run
- On the Raspberry Pi, grab the docker image by running

    `docker pull n33r4j/auto_dog_feeder:v0.1`

    > _Note:  This might take a while._

- Clone this repo in a convenient location.
- Assuming docker is installed on the RPi, run the container with 

    ```
    docker run -it -v ~/<path to cloned repo>/AutoDogFeeder:/AutoDogFeeder \
    -v /dev/video0:/dev/video0 \
    --device-cgroup-rule='c 81:* rmw' \
    n33r4j/auto_dog_feeder:v0.1
    ```

- Update dependencies with

    `cd ~/AutoDogFeeder`

    `rosdep install -yr --from-path src`

    (might be better to put this in an entrypoint.sh or other script.)

- Build the package and source local workspace.

    `colcon build`

    `source install/local_setup.bash`

- Run the launch file. TODO

_Alternatively, if you have the environment already setup(without docker?)_
- Clone repo
- `cd AutoDogFeeder/`
- `chmod +x run_demo.sh`
- `./run_demo.sh` which runs the Camera node.
- You can trigger video recording by switching between publishing a `0` and `1` to the `/camera_mode` topic in a new terminal  with,

`ros2 topic pub -r 10 /camera_mode std msgs/msg/Int8 "{data: 1}"`


# Assumptions
- Only works for small dogs(as in the specs).
- No other pets in the household and minimal other disturbances that might trigger the feeder.
- Powered from a wall outlet(no batteries).
- Gravity based food and water dispensing with minimal jamming.
- Food and water dispensing is coupled.
- The dispenser servo reaches a target in negligible time.
- The dispenser is designed in a way to achieve controlled flow of water and food, i.e. something like a tap.
- The dog is not scared of the automatic feeder or the sounds it makes.
- 

# Systems Diagram
- Raspberry Pi 4
- USB Camera
- PIR Sensor
- Mini servo motor
- LCD Display
- Speaker


![Systems Diagram](images/system_diagram-v1.png)

# Considerations
## Dog Detection Method: 
Several methods are possible depending on factors like _size of the dog_,_no. of dogs(or other pets)_, _possible disturbances_, and _available compute resources_.

1. manual trigger that the dog can be trained to hit(activates near feeding time)
2. camera-based motion detection
3. other sensors (ultrasonic, IR etc)
4. RFID collar/ QR code collar
5. YOLO or pet facial detection

## Camera Placement/Resolution
Depends on the exact purpose of the camera recordings.
- track dog feeding behavior?
- track leftover food/water on dish?
- allow owner to see/interact with dog remotely?

## Video Storage:
- overwrite old recordings(default)?
- backup to a local server/cloud?
- compress videos?

## Feeder:
You may need to dispense water more often than food.

# Testing
- Tests for each node using pytest?
- Create dummy nodes that publish sensor values, call services/actions.
- 
