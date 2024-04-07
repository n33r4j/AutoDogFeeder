# AutoDogFeeder
An automatic dog feeder based on ROS 2(iron).

## Instructions to Run
_(Haven't built an image for RPi4(arm64) yet so this won't work)_
- Grab the docker image from <this_link>.
- Run the container <include self startup script?>

_For now_
- Clone repo
- `cd AutoDogFeeder/`
- `chmod +x run_demo.sh`
- `./run_demo.sh` which runs the Camera node.
- You can trigger video recording by switching between publishing a `0` and `1` to the `/camera_mode` topic in a new terminal  with,

`ros2 topic pub -r 10 /camera_mode std msgs/msg/Int8 "{data: 1}`


# Assumptions
- Only works for small dogs(as in the specs).
- No other pets in the household and minimal other disturbances that might trigger the feeder.
- Powered from a wall outlet(no batteries).
- Gravity based food and water distribution with minimal jamming.
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
- Dog Detection Method: several methods possible depending on size of dog, no. of dogs(or other pets) and possible disturbances.
    - manual trigger that the dog can be trained to hit(activates near feeding time)
    - camera-based motion detection
    - other sensors
    - RFID collar
    - YOLO or pet facial detection
- Camera Placement/Resolution: depends on the exact purpose of the camera recordings.
    - track dog feeding behavior?
    - track leftover food/water on dish?
- Storage: might need to backup to a server or overrite old recordings(might not be ideal).
- 

# Testing
- 