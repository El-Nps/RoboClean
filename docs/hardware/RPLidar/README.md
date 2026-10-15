
# RPLidar A1

[← Raspberry Pi](../RaspberryPi/README.md) | [Hardware →](../README.md)

The project uses an RPLidar A1M8 sensor.

The [handbook](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://gargantua.ai/wp-content/uploads/2024/11/slamtec-triangulation-lidar-rplidar-a1-user-manual.pdf&ved=2ahUKEwi3hZvIwsOTAxUPSPEDHQJGKvEQFnoECBoQAQ&usg=AOvVaw2Y_d8QvB3r5FXhoaSaxHtC) for the RPLIDAR A1 can be found online.

According to our tests, the LiDAR is very demanding during startup. The node is likely to fail if the Raspberry Pi is already occupied with other tasks, either due to power limits or computational load.

## Model Specification

| Property | Value |
| --- | --- |
| S/N | 7CB2ECF0C3E09ED2A0EA98F3EF6A4110 |
| Baudrate | 115200 |
| SYMLINK* | sensors/rplidar |
| VendorID | 10c4 |
| ProductID | ea60 |
| Firmware Ver | 1.29 |
| Hardware Rev | 7 |
| max_distance | 12.0 m |

_**Note:** Marked `*` properties are based on our configuration and could be modified in the future._

[ROS →](../../ros/README.md)
