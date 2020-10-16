# `endpoints_container`

This package contains files necessary to build `endpoints_container` for `navatics_uusv` project. This package is temporarily stored in personal repository, and later will be forked to official `navatics_uusv` repository when it's already matured

## Dependencies

1. `Docker`

2. Eigen (place it in `core_libs/cpp` directory)

## How to use

### 1. Building the Docker Image

To build the docker image, run:

```
docker build -t navatics_endpoints:dev .
```

### 2. Running the Image

Use the startup script with the following command:

```
./startup.sh
```

### 3. Building the ROS packages

In the docker container

```
cd endpoints_ws && colcon build
```

don't forget to source the `setup.bash` through:

```
source ~/endpoints_ws/install/setup.bash
```

### 4. Running Individual Packages

To see how individual packages can be used, please refer to each README.md in the individual package.

### 5. Deploying via `docker-compose`

1. First you would need to build the new docker image. To do so, checkout to `deploy` branch and run:
  ```
  docker build -t navatics_endpoints:deploy .
  ```

2. Ensure that you have `docker-compose` installed by running the following command:
	```
	sudo apt-get install docker-compose
	```
	You can get the latest `docker-compose` installation via [the official installation guide](https://docs.docker.com/compose/gettingstarted/)

3. Navigate to the container root directory and run:
	```
	docker-compose up -d
	```

4. The container should now be running in the background

5. In order to remove the container completely, or to stop it from automatically starting, run:
  ```
  docker kill endpointscontainer_navatics_endpoints_1
  docker rm endpointscontainer_navatics_endpoints_1
  ```
  After that, navigate to `~/navatics_uusv_ws/endpoints_container` and run:
  ```
  docker-compose rm
  ```
  To reactivate, follow the previous instructions.
