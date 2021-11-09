
docker run -it \
    -p 8888:8888 -p 7000-7100:7000-7100 \
    -v $PWD/notebooks:/jupyter/notebooks \
    --user $(id -u):$(id -g) \
    jdm:latest
