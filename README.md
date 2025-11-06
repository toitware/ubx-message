# UBX Message
Support for UBX messages from the UBX data protocol used by ublox GNSS receivers
in the Max-M* series.


## History
This library is a parser of messages from these recievers. The protocol was
developed and improved over time.  The parser was first written for Toit for M8
devices.  Support for 6M (legacy) devices has been added later.


## Specific Field Information

### Integer Time Of Week (iTOW) / GPS Week
This is a millisecond time measurement.  A GPS week starts at 00:00:00 GPS time
on Sunday and lasts exactly 604,800 seconds (7 days).  Week numbering began on
January 6, 1980, which is GPS time "week 0."  It's continuous, Is continuous (no
leap seconds), and is offset from UTC by a constant number of seconds (currently
18 seconds, as of 2025).  It tracks time precisely since 1980-01-06 00:00:00.

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
