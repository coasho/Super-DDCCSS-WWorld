# SIM-M03 carrier integration source checks

## Explicit F-16 / AIM-120D integration

[AFMC, Edwards test squadron returns to Red Flag, 2012-02-13](https://www.afmc.af.mil/News/Article-Display/Article/153727/edwards-test-squadron-returns-to-red-flag)
documents F-16 M6.1+ testing and the new AIM-120D capability during the
January-February 2012 exercise. The earlier runtime source string incorrectly
said 2011; current source now uses the actual publication/event year. Historical
baseline recordings retain their original bytes and are not rewritten.

The explicit `air superiority AIM-120D` loadout in
`aircraft/f16c/payloads.json` selects this integration through the existing native
station, mass, carrier, permission and release path. It preserves the existing
AIM-120C default fit. The exact mixed station arrangement follows the existing
authored AMRAAM/Sidewinder arrangement and is an engineering configuration.

[USAF Air Combat Command AIM-120 fact sheet, 2022-01-27](https://www.acc.af.mil/Portals/92/Docs/Fact%20Sheets%20-%202020%20Update/Facts%20Sheets%202022%20Final/AIM-120_final.pdf)
describes AIM-120D data-link electronics/GPS antennas and F-16 carriage in the
family. These sources do not supply proprietary RF messages, link timing,
actuator or receiver calibration values. Implemented finite transport and sensor
values have their own explicit engineering policy.

## Primary-source platform bounds

[Rosoboronexport R-27ET1](https://roe.ru/pdfs/pdf_4714.pdf) supports the modular
R-27 seeker/motor family. [Rosoboronexport Su-30MK2](https://roe.ru/pdfs/pdf_4668.pdf)
explicitly lists R-27R1/ER1/T1/ET1. It does not, by itself, establish the software
or station configuration of a different exact Su-27 or MiG-29 model. Such a
cross-variant fit cannot be certified from a generic family list alone.

See [the persistent research entry](README.md) for the physical implementation,
original sources and current normal-entrypoint observations.
