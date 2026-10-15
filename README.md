**Note:** This repository is a re-upload of a student project I contributed to, as the original repository is not publicly accessible.

WS 2025
# RoboClean

RoboClean is a student practical offered by the Artificial Intelligence for Programming (AIP) research group at Heidelberg university.
The project revolves around upgrading an outdated [LG HomBot 3.0](https://www.lg.com/de/support/produkt-hilfe/cs-VR6270LVMB.ABKQLGD) vacuum robot.

## Structure

```text
25WS-RoboClean/
 ├─ docs/
 ├─ firmware/
 ├─ frontend/
 ├─ ros2/
 ├─ tests/
 ├─ utils/
 ├─ .env
 ├─ .env.example
 ├─ .gitignore
 ├─ docker-compose.prod.yml
 ├─ docker-compose.yml
 ├─ Dockerfile
 ├─ Dockerfile.prod
 └─ README.md
```

## Prerequisites

This project uses Docker. A running Docker Engine and Docker Compose installation are required — for example, install and run Docker Desktop. For further information and installation instructions, see [Docker Docs](https://docs.docker.com/get-started/get-docker/).

## Quickstart

For local development, the robot is simulated using a headless gazebo server and custom world / robot descriptions. These can be found within the `roboclean`package.

To use the default configuration, copy [`.env.example`](/.env.example) to [`.env`](/.env).

### Map storage configuration

Map files are configured via [`.env`](/.env):

- `MAP_HOST_DIR`: host directory where map files are stored persistently
- `MAP_BASENAME`: base name for map files (`<name>.yaml`, `<name>.pgm`, `<name>.posegraph`)

Example:

```env
MAP_HOST_DIR=/home/your-user/roboclean_maps
MAP_BASENAME=roboclean_map
```

After changing these values, restart the stack:

```bash
docker compose up --build
```

_**Note**: the initial build will take up to 10 minutes._

The frontend will be published to [http://localhost:8080](http://localhost:8080).
Some debugging might require the execution of ROS commands. This can be done with:

```bash
# /entrypoint.sh sources ROS
docker exec -it sim /entrypoint.sh bash
```

_**Note**: information for deployment can be found in the [docs](/docs/deployment/README.md)._

## Documentation

Further information is available in the [docs](/docs/README.md).

[Docs →](/docs/README.md)
