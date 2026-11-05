## Uso
### Lanzar el docker
Antes de lanzar el docker es obligatorio lanzar el siguiente comando:
```
xhost +
```

Luego se levanta el docker desde la carpeta en la que se tiene el archivo kinova-phantom.tar:

```
sudo docker run \
--shm-size=1g \
--privileged \
--ulimit memlock=-1 \
--ulimit stack=67108864 \
--rm \
-it \
--net=host \
-e DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
--device=/dev/ttyACM0:/dev/ttyACM0 \
--name kinova-phantom_container \
--cpuset-cpus=0-3 \
-v ./:/alumno \
-v /run/udev:/run/udev \
kinova-phantom
```

Para abrir más ventanas de terminal con el doker se usará el siguiente comando:
```
sudo docker exec -it kinova-phantom_container /bin/bash
```

### Ejecutar las aplicaciones y simulación
Para poder hacer uso de los archivos en la carpeta dentro del docker debemos movernos a la carpeta alumno del contenedor:
```
cd ../alumno/
```
#### Simulación:
```
ros2 launch gen3_ft_sim gen3_ft_sim.launch.py robot_type:=gen3 dof:=7 gripper:=robotiq_2f_140 use_force_torque:=true
```

#### Simulador de omni:
```
python3 mock_omni.py
```

#### Interfaz de control
```
python3 teleop_gui.py
```
