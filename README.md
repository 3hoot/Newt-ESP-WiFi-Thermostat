# NEWT - ESP WiFi Thermostat

This is a temperature regulator project for my Mechatronics lab, built using an ESP32-MINI. It’s designed to control room temperature via a web interface, using a PID algorithm for stability.

## What it does

* **Temperature Control:** Reads data from a sensor and switches heating/cooling hardware to keep the room at a set temperature. For the purposes of the lab, it just toggles an LED to simulate the output.
* **WiFi Connectivity:** Connects to your local network so you can monitor and adjust settings from any browser.
* **PID Tuning:** Runs an algorithm to tune the heating/cooling response to prevent overshoot and keep the room steady.
* **Web UI:** A simple interface hosted on the ESP32 to check the current temp and change the setpoint.

## How it works

1. **Provisioning:** On the first run, the ESP32 starts a Wi-Fi Access Point (SoftAP). Connect to it with your phone/laptop to enter your home Wi-Fi credentials.
2. **PID Loop:** Once online, the code kicks off the PID loop. It does a quick tuning phase to learn your setup's behavior.
3. **Control:** It maintains the temperature by toggling the output based on the user's setpoint.

## Technical stuff for the lab

* **Hardware:** ESP32-MINI + etc. ADD SENSOR DETAILS HERE
* **Framework:** ESP-IDF (v5.5.2).
* **Provisioning:** Uses the `wifi_provisioning` component (SoftAP mode) to save credentials to NVS so you don't have to re-configure it every time you reboot.