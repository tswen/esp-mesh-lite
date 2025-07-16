- [中文版本](./README_CN.md)

# ESP-Mesh-Lite


ESP-MESH-LITE is a Wi-Fi networking application of [IoT-Bridge](https://github.com/espressif/esp-iot-bridge), based on the **SoftAP + Station** mode, a set of Mesh solutions built on top of the Wi-Fi protocol. ESP-MESH-LITE allows numerous devices (henceforth referred to as nodes) spread over a large physical area (both indoors and outdoors) to be interconnected under a single WLAN (Wireless Local-Area Network). The biggest difference between ESP-MESH-LITE and [ESP-MESH](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/esp-wifi-mesh.html) (also known as ESP-WIFI-MESH) is that ESP-MESH-LITE allows sub-devices in the network to independently access the external network, and the transmission information is insensitive to the parent node, which greatly reduces the difficulty to develop the application layer. ESP-MESH-LITE is self-organizing and self-healing, which means the network can be built and maintained autonomously.

For more information about ESP-MESH-LITE, please refer to [ESP-MESH-LITE Guide](https://github.com/espressif/esp-mesh-lite/blob/master/components/mesh_lite/User_Guide.md).

In the [examples](https://github.com/espressif/esp-mesh-lite/blob/master/examples) directory, demos of some common application scenarios are implemented for users to quickly integrate into their own application projects.

- [examples/mesh_local_control](examples/mesh_local_control): This example simply demonstrates device networking and TCP communication, without complex network applications. Users can develop their own applications based on this example.
- [examples/mesh_wifi_provisioning](examples/mesh_wifi_provisioning): This example demonstrates how to use ESP-MESH-LITE for Wi-Fi provisioning (Wi-Fi Provisioning + Zero Configuration). Users can provision devices and connect them to Wi-Fi networks through the APP.
- [examples/no_router](examples/no_router): This example demonstrates how to use ESP-MESH-LITE for networking and communication between devices without a router.
- [examples/rainmaker/led_light](examples/rainmaker/led_light): This example integrates Mesh functionality into the Rainmaker application. Users can provision devices and connect them to the Rainmaker cloud through the `Nova Home` APP. While the device itself connects to the cloud based on Rainmaker, it can also provide internet access for other wireless devices, forming a network with Mesh-Lite functionality, greatly reducing router load pressure while expanding wireless communication range.
- [examples/wireless_debug](examples/wireless_debug): This example demonstrates how to use ESP-MESH-LITE for wireless debugging. Users can obtain debugging information by sending corresponding commands to the debugging device.
