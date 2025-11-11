# UBX Message
Support for UBX messages from the UBX data protocol used by ublox GNSS receivers
in the Max-M* series.


## History
This library is a parser of messages from these recievers. The protocol was
developed and improved over time.  The parser was first written for Toit for M8
devices.  Support for 6M (legacy) devices has been added later.


## Devices/Version Support
Originally Software version numbers were tied to the protocol.  But at some
point uBlox moved towards maintaining the protocol itself, giving version
nubmers to it. etc.  Features coming along started to have minimum protocol
version requirements.  In order to manage support for different devices with the
same protocol/parser, minimum protocol information has been added to each
message type.  Older devices that don't advertise what protocol version they
support will be assumed this way:

| Device Generation | Example module | SW-Ver (Observed) | UBX protocol equiv. | Notes |
|-|-|-|-|-|
| u-blox 5| `NEO-5Q` / `LEA-5H` | 5.00–5.03 | `13.00` (Assumed) | Earliest unified UBX message set.  Has `UBX-NAV-SOL`, `UBX-NAV-POSLLH`, `UBX-TIMEUTC`, `UBX-NAV-SVINFO`, `UBX-NAV-STATUS` |
| u-blox 6 | `NEO-6M-0-001` / `LEA-6` | 7.00-7.03 | `13.00` (Assumed)	| Adds `UBX-NAV-SVINFO`, `UBX-TIMEUTC`, `UBX-NAV-SOL`.  (still no PROTVER field). |
| u-blox 7 | `NEO-7M-0-000` | 1.00 | `14.00` | Transitional—message formats same as 6-series.  PROTVER string present in observed devices. |
| u-blox 8/M8 | NEO-M8N	| 3.01+ | > `15.00` | First to advertise PROTVER in `UBX-MON-VER` messages.  Adds `NAV-SAT`, `NAT-PVT`, `MON-PATCH` etc. |
| u-blox 9/M9 | NEO-M9N	| 4.xx  | `19.xx` - `23.xx` | Current numbered line. |
| u-blox 10 (M10 Family) | MIA-M10Q, NEO-M10S, MAX-M10S, M10M | 5.xx+ | `27.00` - `27.11` (as of 2025) | Same message framing and checksum; new GNSS, low power, and timing modes.  New high precisions types such as `NAV-HPPOSECEF` and `NAV-HPPOSLLH`. |

The implemented driver will need to determine/assume a suitable protocol version
for the device, and provide any guardrails necessary.  The ubx-message library
will not perform this.  (This will be implmemented with Toit
[ublox-gnss-driver](https://github.com/toitware/ublox-gnss-driver)).

### Proposed logic for downlevel devices
Devices earlier than M8 do not refer to the ubx protocol version when
determining support - they referred to software versions.  Starting with the M8,
the protocol was given its own versioning, which started with `15.0`.  Devices released before this are referred to as 'legacy', and some assumptions are made about the
equivalent protocol version they support

The following logic is proposed for the supported protocol version, if the
device itself does not return the information specifically in its version
extensions (Pseudocode):
```Toit
if monver-message.has-extension "PROTVER":
  // Use extracted protocol version
  protver = monver-message.extension["PROTVER"]

// fallback options where no protocol version exists:
else if UBLOX7-HARDWARE-VERSION == monver-message.hw-version:
  protver = "14.00"
else if UBLOX6-HARDWARE-VERSION == monver-message.hw-version:
  // Assume a u-blox 6
  protver = "13.00"
else:
  // Anything older = minimal UBX core
  protver = 12.0
```

> [!IMPORTANT]
> As this parser was originally written for the M8, the default protocol version
> for all message types is **`15.0`**.
>
> As the message definitions are tested and found to work with earlier chipsets,
> (or ratified with documentation) these numbers will be updated and reduced to
> reflect it.
>
> **It is up to the driver/user to compare these and take action on a mismatch -
> this parser provides these as information only.**


> [!WARNING]
> While the groundwork for the framework of message types and parsing is
> complete, the exact message types you might need may not yet be implemented.
> This is most often largely due to a lack of hardware available to testers...
> Please create an [issue](https://github.com/toitware/ubx-message/issues) or
> contact on [discord](https://discord.gg/Q7Y9VQ5nh2).

## Specific Field Information

### Integer Time Of Week (iTOW) / GPS Week
This is a millisecond time measurement.  A GPS week starts at 00:00:00 GPS time
on Sunday and lasts exactly 604,800 seconds (7 days).  Week numbering began on
January 6, 1980, which is GPS time "week 0."  It's continuous (no
leap seconds) and is offset from UTC by a constant number of seconds (currently
18 seconds as of 2025).  It tracks time precisely since 1980-01-06 00:00:00.

Every new Sunday at midnight starts a new GPS week (week 1, week 2, etc) each
of which runs for seconds (up to 604,799,999 ms in iTOW).

### integer Time Of Week (iTOW) use in messages
All the main UBX-NAV messages (and some other messages) contain an iTOW field
which indicates the GPS time at which the navigation epoch occurred. Messages
with the same iTOW value can be assumed to have come from the same navigation
solution.  Note that iTOW values may not be valid (i.e. they may have been
generated with insufficient conversion data) and therefore it is not recommended
to use the iTOW field for any other purpose.  The original designers of GPS
chose to express time/date as an integer week number (starting with the first
full week in January 1980) and a time of week (often abbreviated to TOW)
expressed in seconds. Manipulating time/date in this form is far easier for
digital systems than the more "conventional" year/month/day, hour/minute/second
representation. Consequently, most GNSS receivers use this representation
internally, only converting to a more "conventional form" at external
interfaces. The iTOW field is the most obvious externally visible consequence of
this internal representation.

# Usage
While different drivers might do this differently (especially if I2c vs. SPI vs.
Serial) the messages operate in the following broad way:
#### Sending a message
To send a message, first create the message using the appropriate constructor,
and then send it to the device:
```Toit
  // Import this library.  (Other driver setup omitted.)
  import ubx-message

  // Instantiate a ubx-mon-ver poll message (no arguments)
  poll-message := ubx-message.MonVer.poll

  // Convert to a byte array, and send to the device:
  adapter.send-packet poll-message.to-byte-array
```
If the message type has an argument, this is specified on the different constructors.
Almost all possible variable names and types are exposed:
```Toit
  // Library/driver setup omitted.

  // Instantiate a ubx-cfg-msg message to request class-id/message-id messages
  // at the rate of xxxx
  classid := ubx-message.Message.NAV
  msgid := ubx-message.NavStatus.ID
  rate := 1
  rate-message := ubx-message.CfgMsg.message-rate --msg-class=classid--msg-id=msgid --rate=rate

  // Convert to a byte array, and send to the device:
  adapter.send-packet rate-message.to-byte-array
```

#### Recieving/Parsing a message
In the background, the parser creates a message using a private constructor

```Toit


```



# Documentation
Note that different documents may exist for your exact version of device (eg differing precisions and feature sets etc).
- ublox 6 Receiver Description [PDF](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf)
- ublox 7 Receiver Description [PDF](https://content.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf)
- ublox 8 Receiver Description [PDF](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
- ublox 9 Receiver Description [PDF](https://content.u-blox.com/sites/default/files/NEO-M9N-00B_DataSheet_UBX-19014285.pdf)
- ublox 10 Receiver Description [PDF](https://content.u-blox.com/sites/default/files/u-blox%20M10-SPG-5.00_InterfaceDescription_UBX-20053845.pdf)
