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

`ros2 topic pub -r 10 /camera_mode std msgs/msg/Int8 "{data: 1}"`


# Assumptions
- Only works for small dogs(as in the specs).
- No other pets in the household and minimal other disturbances that might trigger the feeder.
- Powered from a wall outlet(no batteries).
- Gravity based food and water dispensing with minimal jamming.
- Food and water dispensing is coupled.
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
