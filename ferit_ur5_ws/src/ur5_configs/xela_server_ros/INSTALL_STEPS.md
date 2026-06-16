# Docker start

For each Docker start, you need to do this:

1. Plug in the sensor.
2. **One-time setup — persistent udev rule (run on host):**
    ```sh
    # Verify your device IDs (plug in sensor first)
    udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct" | head -4
    # Expected: idVendor=="0403", idProduct=="6015"  (FTDI FT230X)

    # Copy the rule from the container or create it directly on the host
    sudo cp /etc/udev/rules.d/99-xela-can.rules /etc/udev/rules.d/
    # OR create it manually:
    sudo sh -c 'echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"ttyXELA\", MODE=\"0666\"" > /etc/udev/rules.d/99-xela-can.rules'

    sudo udevadm control --reload-rules
    sudo udevadm trigger

    ls -la /dev/ttyXELA   # verify symlink was created
    ```
    After this, `/dev/ttyXELA` will always point to the XELA sensor regardless of replug order.

3. Run this (now using the persistent symlink):
    ```sh
    ls /dev/ttyXELA                              # verify symlink exists
    slcand -o -s8 -t hw -S 3000000 /dev/ttyXELA
    ifconfig slcan0 up
    ```
4. On the host machine:
    ```sh
    sudo udevadm control --reload-rules
    sudo systemctl daemon-reload
    ```

# Docker Build and Run
For each Docker build and run, you need this:

1. Plug in the sensor.
2. Run this (using the persistent symlink — see one-time setup above):
    ```sh
    ls /dev/ttyXELA                              # verify symlink exists
    slcand -o -s8 -t hw -S 3000000 /dev/ttyXELA
    ifconfig slcan0 up
    ```
3. On the host machine:
    ```sh
    sudo udevadm control --reload-rules
    sudo systemctl daemon-reload
    ```
4. Run:
    ```sh
    xela_conf -c slcan0
    ```

5. Run:
    ```sh
    export PATH="/xela_suite_linux:/etc/xela:${PATH}"
    ```

6. Edit the `xServ.ini` file located at `/etc/xela/xServ.ini` and add the following content:
    ```ini
    [sensor]
    ...
    rotation = 1
    calibration = on
    ```

# Running the Sensor Visualization
When you want to run the sensor visualization, follow these steps:

1. Start the server:
    ```sh
    xela_server -f /etc/xela/xServ.ini
    ```
2. Start the visualization:
    ```sh
    xela_viz -f /etc/xela/xServ.ini
    ```