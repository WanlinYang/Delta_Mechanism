# 12-bit encoder (gimbal encoder)

The [AEAT-6012](https://www.broadcom.com/products/motion-control-encoders/magnetic-encoders/aeat-6012-a06)
magnetic encoder is a 12-bit single-turn encoder produced by AVAGO.
We use three encoders at the same time to measure the angles of the gimbal.

For our control system, the following harware components are needed.

* three AEAT-6012 encoders (one is enough for testing)
* the NU32 microcontroller board

The circuit connection is shown in [12-bit_circuit_diagram.pdf](12-bit_circuit_diagram.pdf).
[gimbal_encoder.c](gimbal_encoder.c) and [gimbal_encoder.h](gimbal_encoder.h) have functions
to read three encoders using an NU32.

The [datasheet of encoder](encoder_datasheet.pdf) is also included.
