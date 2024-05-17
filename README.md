**Set environment:**

```
sudo apt-get update
sudo apt-get upgrade -y

sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg  \
            -o /etc/apt/keyrings/docker.asc
    
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \ 
  https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get upgrade -y

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin \
    docker-compose-plugin nvidia-cuda-toolkit    
sudo apt install x11-xserver-utils

distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update

sudo apt-get install -y nvidia-docker2

sudo docker build -t robotica-inteligente  .

xhost +local:

sudo docker run --shm-size=1g --privileged --ulimit memlock=-1 \
            --ulimit stack=67108864 --rm -it --net=host --gpus all   \ 
            -e DISPLAY=:0 --user=root -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v /dev:/dev --name robotica_container --cpuset-cpus=0-3 \
            -v /home/lev/robotica_practica/src/robotica_inteligente:\
            /home/docker/catkin_ws/src/robotica_inteligente robotica-inteligente
```
