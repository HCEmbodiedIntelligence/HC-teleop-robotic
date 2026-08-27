# Button masks are device-neutral bit sets. The transport adapter owns the
# mapping from vendor buttons to stable bit positions.
uint32 BUTTON_PRIMARY=1
uint32 BUTTON_SECONDARY=2
uint32 BUTTON_GRIP=4
uint32 BUTTON_TRIGGER=8
uint32 BUTTON_MENU=16
uint32 BUTTON_PRIMARY_AXIS_CLICK=32
uint32 BUTTON_PRIMARY_AXIS_TOUCH=64
uint32 BUTTON_SECONDARY_AXIS_CLICK=128
uint32 BUTTON_SECONDARY_AXIS_TOUCH=256
uint32 BUTTON_PRIMARY_TOUCH=512
uint32 BUTTON_SECONDARY_TOUCH=1024

uint32 held_mask
uint32 pressed_mask
uint32 released_mask

# Normalized analogue inputs. Adapters must clamp these to [0, 1].
float32 trigger
float32 grip

# Normalized two-dimensional controller axes in [-1, 1].
float32[2] primary_axis
float32[2] secondary_axis
