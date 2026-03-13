# SynthMachine
A set of Modular synthesizer modules designed around the SSI2130 VCO, SSI2140 Multi-Mode Filter, and the Daisy Seed Microcontroller.

Firmware requires a daisy seed build environment to compile, see daisy seed startup guide (https://daisy.audio/tutorials/cpp-dev-env/), requires updated libdaisy, daisysp, etc... to be cloned from relevant repos.

Very much a prototype.

Credit to timMJN and his Polymorph VCO module: https://github.com/TimMJN/Polymorph-VCO
His designs were helpful reference whilst learning modular synth i/o and standards regarding audio and CV voltage ranges and impedance.

VCO Machine and VCF Machine are designed to use the DAB2130 and DAB2140 breakout boards. As such they have DIP footprints instead of QFN, and do not have decoupling capacitors as they are already included on the breakout.
