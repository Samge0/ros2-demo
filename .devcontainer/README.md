## docker of ros2-demo-dev

### build docker
```shell
docker build . -t samge/ros2-demo-dev-base -f .devcontainer/Dockerfile-dev-base --build-arg PROXY=http://192.168.50.48:7890
```

### upload
```shell
docker push samge/ros2-demo-dev-base
```

### run .devcontainer/devcontainer.json