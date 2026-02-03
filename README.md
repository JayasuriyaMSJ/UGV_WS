# Quick Start Guide - Webots ROS2 Docker

## 3 Simple Steps - Hello

### Step 1: Install Docker (if not already installed)
```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```
**Then logout and login again!**

### Step 2: Setup (Run Once)
```bash
cd /path/to/project
chmod +x docker_*
./docker_setup
```
**What happens:** 
- ✅ Automatically detects if you have GPU
- ✅ Uses GPU if available, falls back to CPU if not
- ✅ No errors either way!
- ⏱️ Takes 10-15 minutes first time

### Step 3: Build and Run
```bash
# Build your ROS2 workspace
./docker_build

# Start simulation
./docker_create

# Stop simulation (when done)
./docker_destroy
```

## That's It! 🎉

### Works On:
- ✅ Ubuntu 20.04/22.04
- ✅ Systems WITH NVIDIA GPU → Uses GPU acceleration
- ✅ Systems WITHOUT GPU → Works perfectly on CPU
- ✅ Laptops, Desktops, Cloud VMs

### Verify It's Working:
```bash
# After docker_create, check running containers
docker compose ps

# Should see 4 containers running:
# - webots_world
# - robot_1, robot_2, robot_3
```

### Troubleshooting:

**GUI doesn't show?**
```bash
xhost +local:docker
export DISPLAY=:0
```

**Need to rebuild?**
```bash
rm -rf ros2_ws/build ros2_ws/install
./docker_build
```

**Start fresh?**
```bash
./docker_destroy
./docker_create
```

### GPU vs CPU - How to Tell?

When you run `./docker_setup`, look for:

**If you see:**
```
✓ NVIDIA GPU detected
✓ GPU support will be enabled
```
→ You're using GPU mode! 🚀

**If you see:**
```
ℹ No NVIDIA GPU detected
  CPU mode will be used
```
→ You're using CPU mode (still works great!) 💻

---

## File Structure You Need:

```
your-project/
├── docker_setup          ← Setup script
├── docker_build          ← Build script
├── docker_create         ← Start script
├── docker_destroy        ← Stop script
├── Dockerfile            ← Image definition
├── docker-compose.yml    ← Container orchestration
└── ros2_ws/
    └── src/              ← Put your ROS2 packages here!
```

