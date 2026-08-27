uint8 SEVERITY_OK=0
uint8 SEVERITY_INFO=1
uint8 SEVERITY_WARNING=2
uint8 SEVERITY_ERROR=3

uint16 OK=0
uint16 INVALID_REQUEST=1
uint16 NOT_CONFIGURED=2
uint16 BUSY=3
uint16 REJECTED=4
uint16 CANCELED=5
uint16 TIMEOUT=6
uint16 STALE=7
uint16 FAULT=8
uint16 UNSUPPORTED=9
uint16 IO_ERROR=10
uint16 INTERNAL_ERROR=255

uint8 severity
uint16 code
string message

# Parallel key/value arrays for small machine-readable diagnostic details.
string[] detail_keys
string[] detail_values
