Here’s a massive, well-organized markdown sheet to fast-track Linux, Docker, ROS2 Humble, PlotJuggler, rosbag2, Isaac Sim, DDS, and all vital/niche debug ops—with actionable command lists, updated options, and real-world tips.

1. Linux Shell Essentials
Command	What it does
pwd	Print working directory
ls -alh	List all (long, human-readable)
cd <dir>	Change directory
cat <file>	Output file content
head/tail <file>	Show file start/end
touch <file>	Create empty file/update timestamp
cp -r <src> <dst>	Copy files or dirs
mv <src> <dst>	Move/rename
rm -rf <dir>	Recursively remove (dangerous)
grep -rin <pat> <path>	Recursive search (line numbers)
sudo <cmd>	Run as superuser
chmod +x <file>	Make file executable
df -h	Disk free (readable)
du -sh *	Dir/file sizes
`ps aux	grep <name>`
kill -9 <pid>	Force kill process
ln -s <src> <dst>	Create symlink
tar -czvf foo.tar.gz dir/	Compress directory
tar -xzvf foo.tar.gz	Extract archive
`history	grep <cmd>`
uname -a	Kernel+system info
top/htop	Monitor live system
watch -n1 nvidia-smi	GPU monitor (great for Isaac Sim)
ifconfig or ip a	Network interfaces
ping <host>	Test connectivity
echo $ENVVAR	Print environment var
alias ll='ls -alF'	Quick alias
[More: GeeksforGeeks], [PhoenixNAP]

2. Docker CLI (Images, Containers, Debug)
Command	What it does
docker ps -a	List all containers
docker images	List local images
docker build -t myimg:tag .	Build image (Dockerfile)
docker run --rm -it <img>	Run interactive container
docker exec -it <cid> bash	Shell into running container
docker logs -f <cid>	Tail logs live
docker stop/rm <cid>	Stop/remove container
docker system prune -af	Prune images/volumes
docker stats	Live resource stats
docker inspect <cid>	Inspect container config
docker pull <img>	Fetch image
docker push <img>	Publish image
docker network ls	List Docker networks
docker volume ls	List Docker volumes
docker-compose up -d	Compose multi-container up
[Ref: GeeksforGeeks Docker], [Docker PDF]

3. ROS2 Humble – Node & System Introspection
Command	What it does
ros2 doctor --report	Full env check (deps, paths)
ros2 node list	List all nodes
ros2 node info <name>	Node pubs/subs/services
ros2 topic list -t	Topics + msg types
ros2 topic echo /topic	Print live topic
ros2 topic hz /topic	Show topic pub frequency
ros2 topic bw /topic	Measure topic bandwidth
ros2 service list -t	List services (+types)
ros2 service call /srv <type> {...}	Manual service calls
ros2 action list -t	List actions (+types)
ros2 action send_goal /action <type> {...}	Send action goal
ros2 param list/get/set	List/get/set node parameters
ros2 pkg list	List installed packages
ros2 launch <pkg> <file.launch.py>	Launch system incl. params
ros2 lifecycle list	Nodes with managed lifecycles
[Ref: ROS2 CLI], [Command lists]

4. ROS 2 Bag & MCAP Commands
Command	What it does
ros2 bag record -a -o name	Record ALL topics
ros2 bag record -s mcap -a -o name	Record to MCAP storage
ros2 bag play <bag>	Replay recorded bag
ros2 bag play <bag> -r 2.0	Replay at 2× speed
ros2 bag play <bag> -l	Loop play
ros2 bag info <bag>	Bag metadata
ros2 bag convert -i in.db3 -o out.yaml	SQLite→MCAP
ros2 bag reindex <bag>	Fix broken index
ros2 bag decompress <bag>	Decompress (if compressed)
Selective record example:

bash
ros2 bag record /topicA /topicB --storage-config-file=cfg.yaml
[Ref: UTP-RoboX], [rosbag cmd]

5. PlotJuggler 3 (Time Series Plotting)
Command	What it does
sudo apt install ros-humble-plotjuggler-ros	Install (ROS2 plugin)
ros2 run plotjuggler plotjuggler	Launch PlotJuggler
GUI: Streaming → Start ROS2 Topic Subscriber	Subscribe to live topics
GUI: File → Open Rosbag2	Load bag for offline plotting
Ctrl+Z/Y	Undo/redo actions
Mouse wheel	Zoom in/out
Ctrl + Left Mouse	Pan plot
Drag with Right Mouse	Plot XY pairs
Add/Remove Row/Column	Flexible multi-plot grids
[Ref: PlotJuggler How-To]

6. DDS, Network, and Fast-DDS Diagnostics
Command	What it does
fastdds discovery -i 0	Start DDS discovery server
fastdds shm clean	Purge SHM files
fastdds xml validate conf.xml	Validate QoS XML
cyclonedds insight	Inspect Cyclone DDS graph
ddsperf	Benchmark DDS config
ROS_LOCALHOST_ONLY=1 ...	Force ROS2 on loopback only
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp	Change DDS vendor
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml	Supply custom config
[Ref: DDS Router Debug]

7. NVIDIA Isaac Sim (Launching, Scripting, Debugging)
Command	What it does
./isaac-sim.sh	Launch Isaac Sim GUI
./isaac-sim.sh --reset-user	Factory reset user configs
./isaac-sim.sh --no-window	Headless mode
./isaac-sim.sh --ext-folder ~/my_exts	Load custom extensions
./isaac-sim.sh --exec "open_stage.py <usd>"	Startup and open scene file
clear_caches.sh	Clear Isaac caches (debug fix)
./python.sh <script.py>	Headless Python launch
isaaclab.sh -s	Run via Isaac Lab helper
[Ref: Isaac Sim install]

8. ROS2 Debugging (GDB, Trace, Graph)
Command	What it does
colcon build --symlink-install	Build workspace/debug symbols
colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo	Build package in debug mode
gdb --args <path_to_node>	Classic GDB on node
ros2 run --prefix 'gdbserver localhost:3000' <pkg> <node>	Attach GDB server
ros2 trace -p	Record LTTng trace
rqt_graph	Live display: node-topic graph
ros2 run tf2_ros tf2_echo <frame1> <frame2>	Inspect TF transforms
colcon test --retest-until-fail 5	Fuzz for flaky tests
[Ref: Juraph on GDB]

9. Niche, Underused, Yet Powerful
Command	What it does
ros2 pkg executables <pkg>	List all executable binaries
colcon graph --dot	Generate workspace dependency graph (dot format)
ros2 control list_hardware_interfaces	List all hardware interfaces [HW/Controller debugging]
ros2 daemon stop && ros2 daemon start	Reset DDS discovery (fixes network weirdness)
[Ref: ROS2 Control]

10. Practical One-Liners
bash
# Record + plot at once
ros2 bag record -o debug /odom & ros2 run plotjuggler plotjuggler

# Replay MCAP bag inside Docker
docker run --network host -v $(pwd):/data ros:humble ros2 bag play /data/bag.mcap

# Headless Isaac Sim (no GUI, remote)
./isaac-sim.sh --no-window --renderer rex --/app/file/isaac/port=8282

# Attach GDB to a node (classic method)
ros2 run --prefix 'gdb -ex run --args' <package> <node>
