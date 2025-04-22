# Docker Deployment for Mobile Sensor Bridge

This directory contains Docker configuration files to easily deploy the Mobile Sensor Bridge for ROS2.

## Prerequisites

- Docker
- Docker Compose
- Git

## Quick Start

1. Clone the repository:
   ```bash
   git clone https://github.com/VedantC2307/ros2-android-sensor-bridge.git
   cd ros2-android-sensor-bridge
   ```

2. Ensure the Docker files are in place:
   - `Dockerfile` - Uses osrf/ros:humble-desktop-full-jammy and performs all setup during build
   - `docker-entrypoint.sh` - Minimal entrypoint that sources ROS environments
   - `docker-compose.yml` - Defines container configuration and command

3. Build and start the Docker container:
   ```bash
   docker-compose build
   docker-compose up
   ```

4. Access the web interface on your mobile device:
   - Open a browser on your mobile device and navigate to `https://<your_computer_ip>:4000`
   - Accept the security warnings about the self-signed certificate
   - Grant permissions for camera, microphone, and AR features when prompted

## Docker Architecture

This Docker setup is optimized for a clean runtime experience:

### Build-time Setup (happens during `docker-compose build`)
- Installation of all system dependencies
- Installation of Node.js dependencies
- Generation of SSL certificates
- Building of the ROS2 package

### Runtime (happens during `docker-compose up`)
- ROS2 environments are sourced
- Only the command specified in docker-compose.yml is executed
- No setup or installation steps performed at runtime

## Docker Files Overview

### Dockerfile
The Dockerfile uses `osrf/ros:humble-desktop-full-jammy` as the base image and:
- Installs system dependencies including Node.js
- Copies your code into the image
- Installs Node.js dependencies
- Generates SSL certificates
- Builds the ROS2 package

### docker-entrypoint.sh
A minimal entrypoint script that:
- Sources the ROS2 environment
- Sources your workspace if available
- Executes the command from docker-compose.yml

### docker-compose.yml
Defines the container configuration:
- Maps port 4000 for the web interface
- Mounts volumes for X11 display
- Sets environment variables
- Specifies the command to run

## Development Workflow

For development:

1. Make changes to your code locally

2. Rebuild the Docker image to incorporate changes:
   ```bash
   docker-compose build
   ```

3. Start the container:
   ```bash
   docker-compose up
   ```

For more interactive development, you can modify the command in docker-compose.yml to start a bash shell instead:
```yaml
command: bash
```

Then connect to the running container:
```bash
docker exec -it mobile_sensor_bridge bash
```

## Troubleshooting

### X11 Display Issues

If you encounter issues with visualization tools like RViz2, make sure your host allows X11 connections:

```bash
xhost +local:docker
```

### Build Issues with Source Command

If you see errors related to the source command in the Dockerfile, ensure you're using bash:

```dockerfile
RUN bash -c "source /opt/ros/humble/setup.bash && command"
```

### Network Connectivity Issues

The container uses bridge networking and exposes port 4000. If you have trouble connecting:
- Check that your firewall allows connections to port 4000
- Verify your computer's IP address is accessible from your mobile device
- Make sure you're using HTTPS (not HTTP) to connect