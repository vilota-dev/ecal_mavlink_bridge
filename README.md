## Example
```bash
./test_timesync serial:///dev/ttyACM0:1500000
```

## Package deb
```bash
cd ecal_mavlink_bridge
mkdir build && cd build
cmake ..
make 
make package
```