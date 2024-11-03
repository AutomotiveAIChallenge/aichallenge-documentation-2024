# How to Operate the Vehicle

## Information Provided to Each Team

The following information will be shared with each team before the event.
| Item | Purpose |
| ---- | ------- |
| Vehicle Number (A1-A8) | Identification number assigned to each team's vehicle |
| Vehicle ECU Username | Username for logging in and SSH connection to the vehicle ECU |
| Vehicle ECU Password | Password for logging in and SSH connection to the vehicle ECU |

## Important Notes

- Please do not rename the `aichallenge-2024` directory under home as scripts are path-dependent.
- ROSBAG recording is not automatic, so please execute the recording command during operation.
- By default, Zenoh communication is set to only receive topics. If you need to change settings during operation, please use either of the following methods:
    - Connect via SSH and execute directly from within the ECU without using Zenoh Bridge
    - Comment out the `allow` section (lines 55-62) in the configuration file (`vehicle/zenoh.json5`) inside `aichallenge-2024` on the ECU
- To connect using a vehicle number, you need to install arp-scan software using the following command:
    - `sudo apt install arp-scan`

## How to Connect to Vehicle ECU

Connect to the ECU via SSH from your local PC using the `CCTB_office_01` Wi-Fi network.

- Install arp-scan on your local PC: `sudo apt update && sudo apt install arp-scan`
- Connect your PC to the `CCTB_office_01` Wi-Fi network (same network as the vehicle ECU).
- On your PC, execute `cd aichallege-2024/remote` to change to the working directory
- Execute `bash connect_ssh.bash <vehicle_name> <username>` on your PC (e.g., `bash connect_ssh.bash A9 aic-team`)
- Enter your PC password if prompted
- Enter the vehicle ECU password when prompted

You should now have access to the vehicle ECU.

If the above commands don't work, please try the following:

- Ask the event staff for the vehicle's `<IP address>`
- Execute `ssh <username>@<IP address>` on your PC

## How to Transfer Autoware to Vehicle ECU

- Please keep the folder name as `aichallenge-2024` due to script path dependencies.

1. An `aichallenge-2024` folder is located under `/home` on the ECU; edit the submit folder within.
2. Transfer aichallenge-2024 from your PC using SCP or VSCode Remote SSH

## Operations After Connecting to Vehicle ECU

### 1. Starting Drivers and Docker Containers

```bash
cd aichallenge-2024
./docker_build.sh dev (only needs to be executed once initially)
bash run_vehicle_tmux.sh
```

The terminal will be split as shown below:
![tmux-image](./images/tmux.png)

- Left ①: Starts ./docker_run dev cpu and enters aichallenge-2024 container
- Right ②: Starts ./docker_run dev cpu and enters aichallenge-2024 container
- Right ③: Vehicle driver software starts
- Right ④: Zenoh bridge starts
- Right ⑤: Nothing specific

The vehicle driver (right ③) and Zenoh bridge (right ④) will start automatically.

### 2. Starting Autoware

Execute within Docker container. By default, use terminal ① on the left or ② on the right.

```bash
cd /aichallenge
./build_autoware.bash (execute only once initially, then as needed when making changes that require rebuilding)
./run_autoware.bash vehicle (autoware starts and is ready)
```

### 3. Recording ROSBAG

Execute within Docker container. By default, use terminal ① on the left or ② on the right.

```bash
cd /aichallenge
source workspace/install/setup.bash
ros2 bag record -a
# If you want to avoid warnings (though they don't affect operation), use this alternative command:
ros2 bag record -a -x "(/racing_kart/.*|/to_can_bus|/from_can_bus)"
# You can also record using this command:
cd /aichallenge
./record_rosbag.bash
```

### 4. When Finishing Operation

Execute the following in terminal ⑤ on the right to stop the container and tmux:

```bash
./stop_vehicle_tmux.sh
```

## For ROS Communication Between Local PC and ECU

- Execute the following on your local PC:

```bash
# Only needed once initially
./docker_build.sh dev
# Enter Docker container
./docker_run.sh dev cpu
# Launch terminator (GUI version of tmux) and split screen by right-clicking
# In one terminator terminal, connect to vehicle via Zenoh
cd /remote
./connect_zenoh.bash <vehicle_number>
# In the other terminator terminal, you can communicate with ECU
# (Example: Launch Rviz)
cd /aichallenge
./run_rviz.bash
```

- Press CTRL+C in the Zenoh terminal to end communication.

## FAQ: Troubleshooting

### Q. ROS communication not working between local PC and ECU / Topic duplication

A. Restart Zenoh on both local PC and ECU

#### Restarting Zenoh on ECU

Stop Zenoh in terminal ⑤ by executing:

```bash
cd vehicle
./kill_zenoh.bash
```

Then restart Zenoh in terminal ④:

```bash
./run_zenoh.bash
```

#### Restarting Zenoh on Local PC

Press CTRL+C in the Zenoh terminal to stop it
Then execute `./connect_zenoh.bash <vehicle_number>` to restart

### Q. Low ROS Topic publishing frequency during Zenoh communication

A. The config file `./vehicle/zenoh.json5` is set for stable communication, default 10Hz.
If needed, you can adjust the publishing frequency by modifying `pub_max_frequencies: ["/*=10"]`.
- After modifying `./vehicle/zenoh.json5`, please run `./docker_build.sh` dev to apply the configuration changes.

### Q. ROS Topics delayed or missing on local PC

A. Topics may be delayed or lost due to communication conditions.

- Try reducing the number of topics displayed on local PC or adjusting publishing frequency
- You can set topic priorities in `./vehicle/zenoh.json5` config file using `pub_priorities: ["/racing_kart/joy=1:express"]`
- Try executing `./remote/network_setting.bash`

### Q. Unsure if inside aichallenge-2024 container

A. A simple check: execute the docker command in the terminal - if you get `bash: docker: command not found`, you're inside Docker.
