# Group-A ESP32 Controller

A minimal ESP32 project (Arduino framework, PlatformIO) used by Group A. It initializes the serial port and blinks the onboard LED while printing messages to the serial monitor.

## 🛠️ Installation
1. Clone the repository:

```bash
git clone https://github.com/sonTungN/esp32-controller-ed3.git
```

2. Create environment and install <b>poetry</b> with Python

```bash
python3 -m venv $VENV_PATH
$VENV_PATH/bin/pip install -U pip setuptools
$VENV_PATH/bin/pip install poetry
```

3. Activate your environment

```bash
source $VENV_PATH/bin/activate
```

4. Install workspace's dependencies

```bash
poetry install
```

5. Deactivate the environment

```bash
deactivate
```
