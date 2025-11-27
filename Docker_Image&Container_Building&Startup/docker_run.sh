#!/bin/bash

# Parámetros con valores por defecto
IMAGE_NAME="kinova-phantom"
CONTAINER_NAME="kinova-phanthom_container"
USER="root"
CPUS="--cpuset-cpus=0-8"
GPUS="--gpus all"
RM=""

OPTIONS="--shm-size=1g --privileged --ulimit memlock=-1 --ulimit stack=67108864 -it --net=host"
ENV_VARS="\
-e DISPLAY=$DISPLAY \
-e QT_X11_NO_MITSHM=1 \
-e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
-e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} \
"

VOLUMES="\
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /dev:/dev \
-v /run/udev:/run/udev \
"

DEVICES="\
--device=/dev/ttyACM0:/dev/ttyACM0 \
"

SHARED_PATH="-v .:/catkin_ws/shared_folder"

RUNTIME="--runtime=nvidia"

# Función para mostrar la ayuda
function show_help() {
    echo "Uso: ./docker_run_command.sh [opciones]"
    echo ""
    echo "Opciones disponibles:"
    echo "  --help -h              Muestra este mensaje"
    echo "  --container-name NAME  Nombre del contenedor"
    echo "  --image-name NAME      Nombre de la imagen"
    echo "  --cpus RANGE           Rango de CPUs (0-8 por defecto)"
    echo "  --no-gpus              Ejecutar sin GPUs"
    echo "  --rm                   Borrar contenedor al salir"
    echo ""
    exit 0
}

# Verificar si se solicita ayuda
if [[ "$1" == "--h" || "$1" == "--help" ]]; then
    show_help
fi

# Permitir modificar parámetros desde la terminal
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --container-name) CONTAINER_NAME="$2"; shift ;;
        --image-name) IMAGE_NAME="$2"; shift ;;
        --cpus) CPUS="--cpuset-cpus=$2"; shift ;;
        --no-gpus) GPUS=""; RUNTIME=""; ;;
        --rm) RM="--rm"; ;;
        *) echo "❌ Opción desconocida: $1"; exit 1 ;;
    esac
    shift
done

# Permitir acceso gráfico
xhost +local:

# Verificar si el contenedor ya existe
if docker ps -a --format "{{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
    echo "📦 El contenedor '$CONTAINER_NAME' ya existe. Reiniciándolo..."
    docker start -ai $CONTAINER_NAME
else
    echo "🚀 Iniciando un nuevo contenedor '$CONTAINER_NAME'..."
    docker run \
        $OPTIONS \
        $RM \
        $CPUS \
        $GPUS \
        $ENV_VARS \
        $VOLUMES \
        $DEVICES \
        $RUNTIME \
        --user=$USER \
        $SHARED_PATH \
        --name $CONTAINER_NAME \
        $IMAGE_NAME
fi

