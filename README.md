# STT-TurtleBot
Kodebase til TurtleBot og PC for 3. semesterprojekt.
## Set up env

```bash
python3 -m venv venv
```

```bash
source venv/bin/activate
```

## install libs
Alle nødvendige Python-pakker er specificeret i `requirements.txt`.  
```bash
pip install -r requirements.txt
```

## Set up TurtleBot
Steps for connecting to TurtleBot and launching the necessary programs and scripts

### SSH
For SSH over local LAN cable, the static IP of the TurtleBot is 192.168.0.20
In terminal of pc run:
```bash
ssh pi@192.168.0.20
```

Password for the TurtleBot is "turtlebot"

For SSH over WiFi. Start Hotspot on your phone with:
* SSID: turtlebot
* Password: turtlebot

Then boot the TurtleBot and connect your PC to the Hotspot.
Run `sudo arp-scan -l` to see the IP of the TurtleBot and run
```bash
ssh pi@<TurtleBot IP>
```

Before running ssh to turtlebot and run:
```bash
ros2 launch mqtt_2_cmd_pkg mqtt_interface.launch.py
```
