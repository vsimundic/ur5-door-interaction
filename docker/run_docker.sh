# RUN DOCKER
docker stop ur5_door_interaction

docker rm ur5_door_interaction

docker run --ipc=host --gpus all --runtime=runc --interactive -it \
--shm-size=10gb \
--env="DISPLAY" \
--volume="$(dirname "${PWD}"):/home/RVLuser" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="/dev:/dev" \
--workdir="/home/RVLuser" \
--privileged \
--net=host \
--name=ur5_door_interaction ur5_di:latest

# docker exec -it ur5_door_interaction bash