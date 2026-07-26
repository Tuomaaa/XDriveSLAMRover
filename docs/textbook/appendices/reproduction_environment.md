# Reproduction Environment

:::{admonition} Appendix status: Initial reference
:class: status status-outline

This page records the supported environment. Exact install and flash procedures
will be expanded alongside the week chapters.
:::

## Hardware and operating systems

| Layer | Reference environment |
| --- | --- |
| MCU | STM32F411CEU6 Black Pill, 100 MHz |
| Firmware tooling | STM32CubeMX project, CMake through the ST VS Code extension |
| SBC | Raspberry Pi 4B |
| SBC operating system | Ubuntu Server 24.04 LTS |
| Robotics middleware | ROS 2 Jazzy |
| CAN | MCP2515/TJA1050, 500 kbit/s |

## Repository workflow

The Raspberry Pi uses a sparse checkout of this repository. Do not edit the Pi
copy directly. The reproducible path is local edit, Git push, Pi pull, build,
and hardware test.

## Build the textbook locally

Use Python 3.12 and install the pinned documentation dependencies:

```console
python -m venv .venv-docs
.venv-docs/Scripts/python -m pip install -r docs/textbook/requirements.txt
.venv-docs/Scripts/sphinx-build -W --keep-going -b html docs/textbook build/textbook
```

On Linux, replace `Scripts` with `bin`. Serve `build/textbook` over HTTP so the
static search index and relative assets behave the same way as GitHub Pages.

## Deployment

Pull requests run the strict build without deployment. A documentation change
merged to `main` uploads the generated HTML through the official GitHub Pages
artifact workflow. The repository's Pages source must be set to **GitHub
Actions** once before the first deployment.
