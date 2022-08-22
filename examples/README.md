# Compiling the code

Make sure you have ARGoS >= 3.0.0-beta52 installed!

Commands:
```shell
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Example SENSORS
```shell
argos3 -c src/experiments/epuck2_sensors.argos
```

## Example SWARM
```shell
argos3 -c src/experiments/epuck2_swarm.argos
```

## Example BATTERY
```shell
argos3 -c src/experiments/epuck2_battery.argos
```

## Example CAMERA
```shell
argos3 -c src/experiments/epuck2_camera.argos
```

## Example ENCODERS
```shell
argos3 -c src/experiments/epuck2_encodes.argos
```
