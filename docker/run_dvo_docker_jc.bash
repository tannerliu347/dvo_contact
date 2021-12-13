container_name=$1

xhost +local:
docker run -it --net=host --privileged --gpus all \
  --user=$(id -u) \
  --workdir=/home/$USER/ \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "/etc/passwd:/etc/passwd:rw" \
  -e "TERM=xterm-256color" \
  --env="QT_X11_NO_MITSHM=1" \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY=$DISPLAY -e USER=$USER \
  -v "/home/chengjerry/Documents/code/duplicate/dvo_contact:/home/$USER/dvo_contact/" \
  --device=/dev/dri:/dev/dri \
  --name=${container_name} \
  dvo/visualization:latest

