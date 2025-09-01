# ESP32 Flashing Guide

## Prerequisites
- Docker installed
- Connect the ESP32. Check the esp port, you can use : `ls /dev/ttyUSB*`
- If it is not /dev/ttyUSB0, edit docker-compose.yml and update this line:
  ```
  devices:
    - /dev/ttyUSB*
  ```
- Then use that port in idf.py:
  idf.py -p /dev/ttyUSB* app-flash monitor

## Steps

1. Start the Docker environment:
  ```bash
  ./start_docker.sh
  ```
2. Inside the running container (you will be attached automatically):
  ```bash
  idf.py app-flash monitor
  ```

## Common Tips
- Exit monitor: Ctrl+]
- Full clean + flash:
  ```bash
  idf.py fullclean app-flash monitor
  ```
- If multiple serial ports: set
  ```bash
  idf.py -p /dev/ttyUSB0 app-flash monitor
  ```