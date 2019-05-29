#!/usr/bin/env bash
if [ $# -gt 0 ]; then
    if [ "$1" == "same" ]; then
        txdocker exec -it guiding-robot bash
    else
        txdocker run --name guiding-robot \
            --rm -it --net=host --privileged \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix/:/tmp/.X11-unix \
            -v /dev:/dev \
            -v /etc/localtime:/etc/localtime:ro \
            -v /var/run/docker.sock:/var/run/docker.sock \
            -v /home/$USER/guiding-robot-code:/root/guiding-robot-code \
            -v ~/.bashrc:/root/.bashrc \
            -w /root/guiding-robot-code coolcat647/duckiepond:"$1"_pytorch
	fi
else
	echo "please provide docker tag name."
fi
