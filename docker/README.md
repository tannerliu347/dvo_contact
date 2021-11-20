## DVO docker
This docker file sets up the environment for DVO.


### How to build the docker image from `Dockerfile`?
If you want to make any changes to this docker image, edit the `Dockerfile`. If any changes happends, remember to update the `LABEL version` inside. 
Then to build the docker image so that you can run it, type `docker build --tag dvo/visualization  . `. Here `dvo/visualization` is the name of the image


### How to create a container after building this dockerfile?
`bash run_cuda_docker.bash [container_name]`



### How to start the container after a restart?
`docker start [container_name]`

### How to open a terminal in the docker?
`docker exec -it [container_name] /bin/bash`

If you need root priviledge:

`docker exec -u root -it [container_name] /bin/bash`
