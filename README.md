# Minuet firmware

[Minuet](https://github.com/brown-studios/minuet) upgrades your [MAXXAIR Maxxfan](https://www.maxxair.com/products/fans/maxxfan-deluxe/) with a brushless DC motor, home automation features, and accessories.

This repository contains the Minuet firmware based on ESPHome.  You are free to customize the firmware as you like.

*Refer to the main [Minuet repository](https://github.com/brown-studios/minuet) for documentation and hardware design files.*

## ESPHome configuration YAML

[ESPHome](https://esphome.io/) is an open-source firmware framework.

The Minuet firmware is built from ESPHome configuration YAML files.

The top-level configuration file is [minuet.yaml](./minuet.yaml).  It configures important settings such as the WiFi connection, hardware version, included accessories, and extra components you may have added.  To customize the Minuet firmware, make a copy of `minuet.yaml` and modify it according to the instructions in the comments within the file.

The [minuet](./minuet) directory contains several more configuration YAML files and some C++ code that implements the core of the Minuet firmware.  If possible, please avoid modifying the files in the `minuet` directory because that will make it more difficult for you to upgrade to newer firmware versions.

### How to modify the firmware

It gets easier the second time you do it.

- Familiarize yourself with the [ESPHome](https://esphome.io/) tools you will be using to compile and run the code.
- Download the most recent contents of this repository.
- Make a copy of `minuet.yaml` with a name of your choice, such as `my_minuet.yaml`.
- Ensure that the `minuet` directory is next to `my_minuet.yaml`.  If you are using the ESPHome Builder add-on for Home Assistant, make sure to copy the `minuet` directory into the `/homeassistant/esphome` directory together with `my_minuet.yaml`.
- Compile the unmodified firmware to confirm that your development environment works correctly before you start making changes.
- Edit `my_minuet.yaml` to your heart's content.
- Compile the modified firmware.
- Flash the firmware to the device using an over-the-air firmware update or a USB cable.
- Watch the debug logs to see what's happening.

> [!NOTE]
> To reset into the bootloader when flashing the firmware over USB, press and hold the `BOOT` button then tap `RESET`.  This step should only be needed the first time the device is being flashed.  The device must be powered by a 12 V (nominal) supply to operate; it is not powered from USB.

### Supporting hardware expansion

When you attach additional hardware to Minuet via the `QWIIC` connector or `EXPANSION` port, you will need to add components to the firmware to support the expansion.

Follow the instructions in `minuet.yaml` and either uncomment and configure the package needed for your hardware or add ESPHome components to the `my_package` package as shown near the end of the file.

### Packages

The YAML configuration is subdivided into [packages](https://esphome.io/components/packages/) of components for ease of maintenance.  Each package declares a group of components related to a particular subsystem such as the fan motor driver, keypad, thermostat, or an accessory.  If you're adding a new subsystem, then you should create a new package for it to keep the components tidy.

Some files contain collections of packages when one package just isn't enough.

### Identifiers

To prevent conflicts with end-user firmware customization, all Minuet component identifiers in YAML have the `minuet_` prefix.

Similarly, Minuet C++ declarations reside in the `minuet` namespace and preprocessor macros have the `MINUET_` prefix.

## Contributing to the Minuet firmware

You can do a lot of cool stuff with Minuet and ESPHome like plugging in I2C sensors, making accessories, and adding home automation features to enrich your living space.

Feel free to suggest ideas for improvement to Minuet using the Github issue tracker.  You can also send pull requests to fix bugs and to add features.  We recommend consulting the developers for guidance if you have big changes in mind.  We reserve the right to not accept contributions for any reason.

> [!IMPORTANT]
> We will not accept contributions that have been substantially produced with generative AI tools.  As an open-source project, we expect you to warrant that your contributions are your own work and that they comply with all applicable licenses.

We look forward to hearing your thoughts and seeing the cool stuff that you make with Minuet!

## External components

Minuet uses these external components for some of its functions.  You can also use them in your own projects.

- [esphome-maxxfan-protocol](https://github.com/brown-studios/esphome-maxxfan-protocol): Maxxfan infrared remote control protocol
- [esphome-mcf8316](https://github.com/brown-studios/esphome-mcf8316): MCF8316 brushless DC motor driver with field-oriented control

## Notice

The Minuet software, documentation, design, and all copyright protected artifacts are released under the terms of the [MIT license](LICENSE).
