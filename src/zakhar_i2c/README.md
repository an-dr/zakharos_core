# Zakhar I2C device

This file describes the logic of functioning of slave devices used ar Zakhar's systems

## Registers

Minimal register map:

```none
0x00 : `REG_CMD` Command register. The device is reading commands from here
0x01 : REG_MODEMode register. If device support forking in several modes, this register should be used to indicate it.
```

## Logic

### Command sending workflow for a master

- Master writes `0xFF` to the `REG_CMD`
- Master check if the value at the `REG_CMD` is `0xFF` if not - retry
- Master writes a command to the `REG_CMD` (command is any value in the range `0x01`...`0xFE`)
- Master is writing the value until `REG_CMD` become 0x00
- When the master read `0x00` - it is considered that the command is sent successfully.

### Command handling workflow for a slave

- Slave is ignoring `0x00` and `0xFF` at `REG_CMD`
- If it founds any other value it tries to execute a related function. If there is no function connected to the value at `REG_CMD` - slave just pass this step
- After the execution (or passing) slave writes `0x00` to the `REG_CMD`
