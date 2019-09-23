
## Getting Started

### Compatibility

Works with OSCC `v1.2.1` and up.

### Linux Dependencies

- `python3` (`sudo apt install python3`)

### Windows Dependencies

- `python3` ([https://www.python.org/downloads/windows/](https://www.python.org/downloads/windows/))
- CAN driver ([Kvaser](https://www.kvaser.com/developer/canlib-sdk/), [PCAN](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1), etc.)

### Building

Install this package's dependencies with the following command:

```bash
python3 setup.py install --user
```

## Usage

`read_and_control.py (-V <vehicle>) [-hdelv] [-b <bustype>] [-c <channel>]`

### Options

```bash
Options:
    -h --help                            Display this information
    -V <vehicle>, --vehicle <vehicle>    Specify your vehicle. Required.
                                         (kia_soul_ev / kia_soul_petrol / kia_niro)
    -d --disable                         Disable modules only, no further checks (overrides enable)
    -e --enable                          Enable modules only, no further checks
    -l --loop                            Repeat all checks, run continuously
    -b --bustype <bustype>               CAN bus type [default: socketcan_native]
                                         (for more see https://python-can.readthedocs.io/en/2.1.0/interfaces.html)
    -c <channel>, --channel <channel>    Specify CAN channel, [default: can0]
    -v --version                         Display version information
```

### Usage Notes

#### Linux

On a Linux system `socketcan` is a default so the `oscc-check.py` can rely on the default settings
for `bustype` and `channel`. After initializing the socketcan interface with:

```bash
 sudo ip link set can0 type can bitrate 500000
 sudo ip link set up can0
```

you can run:

```bash
# Default Linux usage for Kia Soul EV
python3 read_and_control -V kia_soul_ev
```

#### Windows

On a Windows system `socketcan` is not available so the `bustype` and `channel` must be specified.

If you've installed the Kvaser SDK you need to run:

```bash
# Default Kvaser CANlib usage for Kia Soul Ev
python read_and_control -c 0 -b kvaser -V kia_soul_ev
```
