This is a UAVCAN reader and writer designed to run on an ArduPilot board. It can
be used to read and write over a UAVCAN bus.

To build and upload for a Pixhawk style board run this:

```
 ./waf configure --board fmuv3 
 ./waf --target examples/UAVCAN_read_write --upload
```
 
then connect on the USB console. You will see 1Hz packet stats like
this:

```
```

<!-- note that the code requires you to add new msg types you want to
see. Look for the MSG_CB() and START_CB() macros in the code -->
