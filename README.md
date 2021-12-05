# iiwa_rock_climbing

### Setup
+ Follow the instructions on this repo to set up a docker container to run the simulator
https://github.com/cameronwp/jupyter-drake-meshcat

+ To start the container and run the code in this repo, cd to the base folder of this repo and run this command:
```
docker run -it \
    -p 8888:8888 -p 7000-7100:7000-7100 \
    -v $PWD:/jupyter/iiwa_rock_climbing \
    --user $(id -u):$(id -g) \
    jdm:latest
```
