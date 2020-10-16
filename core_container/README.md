# `core_container`

This package contains files necessary to build `core_container` for `navatics_uusv` project. This package is temporarily stored in personal repository, and later will be forked to official `navatics_uusv` repository when it's already matured

## Dependencies

1. `Docker`

2. Eigen (place it in `core_libs/cpp` directory)

## How to use

### 1. Building the Docker Image

To build the docker image, run:

```
docker build -t navatics_core:dev .
```

### 2. Running the Image

Use the startup script with the following command:

```
./startup.sh
```

### 3. Building the ROS packages

In the docker container

```
cd core_ws && colcon build
```

don't forget to source the `setup.bash` through:

```
source ~/core_ws/install/setup.bash
```

### 4. Running Individual Packages

To see how individual packages can be used, please refer to each README.md in the individual package.
