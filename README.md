# 🧠 ROS2 + Linux + Docker + Isaac Sim Debug & Ops Cheat Sheet

A massive, well-organized markdown sheet to fast-track Linux, Docker, ROS2 Humble, PlotJuggler, rosbag2, Isaac Sim, DDS, and all vital/niche debug ops—with actionable command lists, updated options, and real-world tips.

---

## 1. 🐧 Linux Shell Essentials

| Command                     | What it does                       |
| --------------------------- | ---------------------------------- |
| `pwd`                       | Print working directory            |
| `ls -alh`                   | List all (long, human-readable)    |
| `cd <dir>`                  | Change directory                   |
| `cat <file>`                | Output file content                |
| `head/tail <file>`          | Show file start/end                |
| `touch <file>`              | Create empty file/update timestamp |
| `cp -r <src> <dst>`         | Copy files or dirs                 |
| `mv <src> <dst>`            | Move/rename                        |
| `rm -rf <dir>`              | Recursively remove (dangerous)     |
| `grep -rin <pat> <path>`    | Recursive search (line numbers)    |
| `sudo <cmd>`                | Run as superuser                   |
| `chmod +x <file>`           | Make file executable               |
| `df -h`                     | Disk usage (human-readable)        |
| `du -sh *`                  | Dir/file sizes                     |
| `ps aux \| grep <name>`     | Process lookup                     |
| `kill -9 <pid>`             | Force kill process                 |
| `ln -s <src> <dst>`         | Create symlink                     |
| `tar -czvf foo.tar.gz dir/` | Compress directory                 |
| `tar -xzvf foo.tar.gz`      | Extract archive                    |
| `history \| grep <cmd>`     | Command recall                     |
| `uname -a`                  | Kernel+system info                 |
| `top` / `htop`              | Live system monitor                |
| `watch -n1 nvidia-smi`      | GPU monitor (great for Isaac Sim)  |
| `ifconfig` / `ip a`         | Network interfaces                 |
| `ping <host>`               | Test connectivity                  |
| `echo $ENVVAR`              | Print env var                      |
| `alias ll='ls -alF'`        | Quick alias                        |

---

## 2. 🐳 Docker CLI (Images, Containers, Debug)

| Command                           | What it does                 |
| --------------------------------- | ---------------------------- |
| `docker ps -a`                    | List all containers          |
| `docker images`                   | List local images            |
| `docker build -t myimg:tag .`     | Build image (Dockerfile)     |
| `docker run --rm -it <img>`       | Run interactive container    |
| `docker exec -it <cid> bash`      | Shell into container         |
| `docker logs -f <cid>`            | Tail logs                    |
| `docker stop <cid>` / `docker rm` | Stop/remove container        |
| `docker system prune -af`         | Clean up everything          |
| `docker stats`                    | Live resource stats          |
| `docker inspect <cid>`            | Inspect container            |
| `docker pull` / `docker push`     | Download/upload image        |
| `docker network ls` / `volume ls` | Show networks/volumes        |
| `docker-compose up -d`            | Run multi-container services |

---

## 3. 🤖 ROS2 Humble CLI – Nodes & Introspection

| Command                             | What it does            |
| ----------------------------------- | ----------------------- |
| `ros2 doctor --report`              | Full environment report |
| `ros2 node list` / `node info`      | Nodes & their info      |
| `ros2 topic list -t` / `echo`       | Topics & live view      |
| `ros2 topic hz` / `bw`              | Frequency & bandwidth   |
| `ros2 service list -t` / `call`     | List/call services      |
| `ros2 action list -t` / `send_goal` | Action goals            |
| `ros2 param list/get/set`           | Node parameters         |
| `ros2 pkg list`                     | List installed packages |
| `ros2 launch <pkg> <launch.py>`     | Launch nodes            |
| `ros2 lifecycle list`               | Lifecycle node status   |

---

## 4. 🎥 rosbag2 + MCAP

| Command                                    | What it does       |
| ------------------------------------------ | ------------------ |
| `ros2 bag record -a -o name`               | Record all topics  |
| `ros2 bag record -s mcap -a -o name`       | Record to MCAP     |
| `ros2 bag play <bag>`                      | Replay             |
| `ros2 bag play <bag> -r 2.0`               | Replay 2× speed    |
| `ros2 bag play <bag> -l`                   | Loop playback      |
| `ros2 bag info <bag>`                      | Show metadata      |
| `ros2 bag convert -i in.db3 -o out.yaml`   | SQLite to MCAP     |
| `ros2 bag reindex` / `ros2 bag decompress` | Repair/recover bag |

### 🎙️ Selective `rosbag2` Recording

```bash
ros2 bag record /topicA /topicB --storage-config-file=cfg.yaml
```

---

## 5. 📊 PlotJuggler (Time Series Plotting)

| Command                                       | What it does                |
| --------------------------------------------- | --------------------------- |
| `sudo apt install ros-humble-plotjuggler-ros` | Install plugin              |
| `ros2 run plotjuggler plotjuggler`            | Launch GUI                  |
| `GUI > Streaming`                             | Subscribe to live topics    |
| `GUI > File > Open Rosbag2`                   | Plot offline                |
| `Mouse + keys`                                | Zoom, pan, undo, drag, plot |

---

## 6. 📡 DDS, Fast DDS, and Network Debugging

| Command                                      | What it does              |
| -------------------------------------------- | ------------------------- |
| `fastdds discovery -i 0`                     | Start discovery server    |
| `fastdds shm clean`                          | Clear shared memory       |
| `fastdds xml validate conf.xml`              | Validate QoS settings     |
| `cyclonedds insight`                         | DDS graph viewer          |
| `ddsperf`                                    | Benchmark DDS configs     |
| `ROS_LOCALHOST_ONLY=1`                       | Localhost mode            |
| `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | Change DDS vendor         |
| `export CYCLONEDDS_URI=...`                  | Use custom Cyclone config |

---

## 7. 🧪 NVIDIA Isaac Sim (Debug, Headless, Scripts)

| Command                        | What it does           |
| ------------------------------ | ---------------------- |
| `./isaac-sim.sh`               | Launch Isaac GUI       |
| `--reset-user`                 | Reset user config      |
| `--no-window`                  | Headless mode          |
| `--ext-folder ~/my_exts`       | Load custom extensions |
| `--exec "open_stage.py <usd>"` | Open scene file        |
| `clear_caches.sh`              | Clear Isaac cache      |
| `./python.sh script.py`        | Headless Python        |
| `isaaclab.sh -s`               | Launch Isaac Lab       |

---

## 8. 🛠️ ROS2 Debugging & GDB

| Command                                        | What it does          |
| ---------------------------------------------- | --------------------- |
| `colcon build --symlink-install`               | Build w/ symlinks     |
| `colcon build --packages-select <pkg> ...`     | Debug build           |
| `gdb --args <node>`                            | Launch node under GDB |
| `ros2 run --prefix 'gdbserver localhost:3000'` | Remote GDB debugging  |
| `ros2 trace -p`                                | LTTng trace           |
| `rqt_graph`                                    | Visual node graph     |
| `tf2_echo frame1 frame2`                       | Inspect TF frames     |
| `colcon test --retest-until-fail 5`            | Fuzz flaky tests      |

---

## 9. 🧩 Niche But Powerful

| Command                                 | What it does             |
| --------------------------------------- | ------------------------ |
| `ros2 pkg executables <pkg>`            | List all binaries        |
| `colcon graph --dot`                    | Build dependency graph   |
| `ros2 control list_hardware_interfaces` | Show hardware interfaces |
| `ros2 daemon stop && ros2 daemon start` | Restart DDS daemon       |

---

## 10. 💡 Practical One-Liners

```bash
# Record + plot simultaneously
ros2 bag record -o debug /odom & ros2 run plotjuggler plotjuggler

# Replay MCAP bag inside Docker
docker run --network host -v $(pwd):/data ros:humble ros2 bag play /data/bag.mcap

# Headless Isaac Sim with renderer
./isaac-sim.sh --no-window --renderer rex --/app/file/isaac/port=8282

# Attach GDB to a ROS2 node
ros2 run --prefix 'gdb -ex run --args' <package> <node>
```
