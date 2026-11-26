#!/bin/bash
# Añade las cámaras a la simulación YA lanzada y publica sus imágenes en ROS 2.
# Uso (dentro del contenedor, desde /alumno):   bash lanzar_camaras.sh
# Dejar esta terminal abierta: el puente gz -> ROS se ejecuta aquí.

DIR="$(cd "$(dirname "$0")" && pwd)"

# 1) Crear el modelo de las cámaras en el mundo
ros2 run ros_gz_sim create -file "$DIR/camaras.sdf" -name camaras || {
  echo "ERROR: no se pudo crear el modelo. ¿Está Gazebo en marcha?"; exit 1; }

sleep 2

# 2) Comprobar que Gazebo está renderizando las cámaras
if ! gz topic -l | grep -q "/cam_top/image"; then
  echo "AVISO: Gazebo no publica /cam_top/image."
  echo "  Falta el sistema 'Sensors' en el mundo. Añade dentro de <world> del .sdf del mundo:"
  echo '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
  echo '      <render_engine>ogre2</render_engine>'
  echo '    </plugin>'
  echo "  y vuelve a compilar/lanzar la simulación."
fi

# 3) Puente Gazebo -> ROS 2 (las imágenes salen como sensor_msgs/Image)
ros2 run ros_gz_bridge parameter_bridge \
  /cam_top/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /cam_side/image@sensor_msgs/msg/Image[gz.msgs.Image
