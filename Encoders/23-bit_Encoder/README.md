# 23-bit encoder (spring encoder)

The [AS38-H39E](https://www.broadcom.com/products/motion-control-encoders/absolute-encoders/single-turn-encoders/as38-h39e-series)
optical encoder offers 23-bit single-turn and 16-bit multi-turn counts, hence a combined 39-bit high resolution.
We use it to measure the rotation angle a torsion spirng.

To test the encoder, the following hardware components are needed.

* the AS38-H39E encoder
* the NU32 microcontroller board
* the [SN65LBC179](https://www.mouser.com/ProductDetail/Texas-Instruments/SN65LBC179P/?qs=sGAEpiMZZMtk5jbcouIbS3K62zSAo8dCUewWBYHOb2w%3d) transceiver

The circuit connection is shown in [23-bit_circuit_diagram.pdf](23-bit_circuit_diagram.pdf)

[op_encoder.c](op_encoder.c) and [op_encoder.h](op_encoder.h) have functions to read the 23-bit-precision angle within 360 degrees using an NU32.
[main.c](main.c) has some examples.

The datasheets of [encoder](encoder_datasheet.pdf) and [transceiver](transceiver_datasheet.pdf) are also included.
