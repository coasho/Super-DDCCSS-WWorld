# Stores: what the pipeline built

Every store on the cards in `assets/stores`, through hangar's stages ([hangar.md](hangar.md#stores)) - written by
`python -m hangar stores --force --jobs 1 --record docs/hangar-stores-build.md`. Each row is one store's last run: its model for
the viewer (`aircraft/<name>/<name>.glb`) and its JSBSim flight model (`aircraft/stores/<name>/<name>.xml`; a
missile's motor and nozzle files beside it, `Engines/`).

- **Run:** 2026-10-10 18:01:35 to 2026-10-10 18:08:12, hangar 0.1.0, Windows-11-10.0.26300-SP0, 16 logical processors, one store at a time.
- **Made again:** every output - each model meshed anew, each flight model's files removed and written (`--force`).
- **Model time:** the mesher's, the paint's and the file's writing (a cargo store's box is drawn, not meshed).
- **XML time:** the build-up from the card and the design, the text and the file's writing - the engine files with it.

| store | class | triangles | model time s | model size kB | XML time ms | XML size kB | motor files kB |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 463L pallet, loaded (`463lpallet`) | cargo | 72 | 0.001 | 8.3 | 0.4 | 8.3 | - |
| 600 US gal external tank (A-10) (`a10tank600gal`) | tank | 5996 | 2.927 | 215.1 | 0.4 | 13.0 | - |
| AIM-120C-7 AMRAAM (`aim120c`) | aam | 83092 | 4.108 | 2534.5 | 1.5 | 21.0 | 0.8 |
| AIM-120D AMRAAM (`aim120d`) | aam | 83092 | 4.117 | 2534.7 | 1.5 | 21.0 | 0.8 |
| AIM-7M Sparrow (`aim7m`) | aam | 102230 | 4.380 | 3465.9 | 1.5 | 21.1 | 0.8 |
| AIM-9M Sidewinder (`aim9m`) | aam | 87164 | 3.859 | 2762.7 | 1.6 | 21.1 | 0.7 |
| AIM-9X Sidewinder (`aim9x`) | aam | 42044 | 3.718 | 1512.3 | 1.6 | 21.0 | 0.7 |
| AN/AAQ-28 LITENING (`anaaq28`) | pod | 5986 | 2.697 | 211.9 | 0.4 | 12.9 | - |
| AN/ALQ-131(V) (`analq131`) | pod | 5976 | 4.120 | 212.1 | 0.4 | 12.9 | - |
| AN/ALQ-184(V) (`analq184`) | pod | 5972 | 4.503 | 210.1 | 0.4 | 12.9 | - |
| AN/ALQ-99 (`analq99`) | pod | 5990 | 3.832 | 212.4 | 0.4 | 12.9 | - |
| AN/ASQ-228 ATFLIR (`anasq228`) | pod | 5984 | 1.726 | 210.4 | 0.4 | 12.9 | - |
| AIM-132 ASRAAM (`asraam`) | aam | 48826 | 4.625 | 1676.8 | 1.4 | 21.0 | 0.8 |
| CDS bundle (A-22) (`cdsa22`) | cargo | 72 | 0.001 | 8.4 | 0.4 | 8.3 | - |
| Damocles (`damocles`) | pod | 5980 | 2.859 | 211.7 | 0.5 | 12.9 | - |
| 600 US gal external tank (F-22) (`f22tank600gal`) | tank | 5996 | 2.918 | 215.5 | 0.4 | 13.0 | - |
| FPU-11/A 480 US gal tank (`fpu11`) | tank | 5990 | 3.368 | 216.8 | 0.5 | 13.0 | - |
| FPU-8/A 330 US gal tank (`fpu8`) | tank | 5988 | 2.972 | 215.1 | 0.4 | 13.0 | - |
| 1,100 L external tank (Gripen) (`gripentank1100l`) | tank | 5992 | 2.936 | 215.8 | 0.4 | 13.0 | - |
| IRIS-T (`irist`) | aam | 48110 | 4.894 | 1733.9 | 1.5 | 21.0 | 0.7 |
| 1,600 L external tank (J-10) (`j10tank1600l`) | tank | 5988 | 2.867 | 213.4 | 0.4 | 13.0 | - |
| 800 L external tank (J-10) (`j10tank800l`) | tank | 6000 | 2.942 | 215.1 | 0.4 | 13.0 | - |
| R.550 Magic 2 (`magic2`) | aam | 74120 | 4.283 | 2652.3 | 1.6 | 21.0 | 0.7 |
| Meteor (`meteor`) | aam | 43312 | 4.377 | 1508.2 | 1.5 | 21.0 | 0.8 |
| MICA EM (`micaem`) | aam | 58218 | 6.130 | 1916.2 | 1.5 | 21.0 | 0.7 |
| MICA IR (`micair`) | aam | 58330 | 6.162 | 1925.4 | 1.5 | 21.0 | 0.7 |
| Mk 82 (`mk82`) | bomb | 9980 | 2.206 | 423.8 | 47.6 | 19.7 | - |
| PL-10 (`pl10`) | aam | 56118 | 5.296 | 1850.6 | 1.5 | 21.0 | 0.7 |
| PL-12 (`pl12`) | aam | 92526 | 4.969 | 2858.7 | 1.5 | 21.0 | 0.8 |
| PL-15 (`pl15`) | aam | 48264 | 3.515 | 1584.1 | 1.4 | 21.0 | 0.8 |
| PL-8 (`pl8`) | aam | 114090 | 8.359 | 4326.3 | 1.7 | 21.0 | 0.7 |
| PTB-1150 (`ptb1150`) | tank | 5992 | 2.960 | 214.8 | 0.4 | 12.9 | - |
| PTB-1500 (`ptb1500`) | tank | 5992 | 2.940 | 215.6 | 0.4 | 12.9 | - |
| PTB-2000 (`ptb2000`) | tank | 5974 | 2.930 | 213.1 | 0.5 | 13.0 | - |
| PTB-800 (`ptb800`) | tank | 6000 | 2.905 | 214.9 | 0.4 | 12.9 | - |
| R-27ER (`r27er`) | aam | 109066 | 6.181 | 3769.4 | 1.6 | 21.0 | 0.8 |
| R-27ET (`r27et`) | aam | 107396 | 5.724 | 3699.8 | 1.8 | 21.0 | 0.8 |
| R-27R (`r27r`) | aam | 107510 | 6.094 | 3865.4 | 1.8 | 21.0 | 0.7 |
| R-27T (`r27t`) | aam | 114120 | 6.902 | 4104.9 | 2.1 | 21.0 | 0.7 |
| R-37M (`r37m`) | aam | 92614 | 6.348 | 2739.2 | 1.5 | 20.9 | 0.9 |
| R-60M (`r60m`) | aam | 75664 | 2.378 | 2262.4 | 1.6 | 21.0 | 0.7 |
| R-73 (`r73`) | aam | 105128 | 5.292 | 3590.5 | 1.7 | 21.0 | 0.7 |
| R-77M (`r77m`) | aam | 121614 | 15.421 | 4666.2 | 1.5 | 21.0 | 0.8 |
| RPL-522 (`rpl522`) | tank | 5996 | 2.536 | 215.4 | 0.4 | 12.9 | - |
| RPL-541/542 (`rpl541`) | tank | 5996 | 3.059 | 214.3 | 0.4 | 13.0 | - |
| RPL-701 (`rpl701`) | tank | 5992 | 2.877 | 215.3 | 0.4 | 12.9 | - |
| RPL-751 (`rpl751`) | tank | 5974 | 2.937 | 213.8 | 0.5 | 13.0 | - |
| Super 530D (`super530d`) | aam | 91840 | 5.870 | 2829.1 | 1.7 | 21.0 | 0.8 |
| 300 US gal centerline tank (F-16) (`tank300gal`) | tank | 5992 | 2.959 | 212.6 | 0.4 | 13.0 | - |
| 370 US gal external tank (F-16) (`tank370gal`) | tank | 5990 | 2.811 | 214.2 | 0.4 | 13.0 | - |
| 610 US gal external tank (F-15) (`tank610gal`) | tank | 5992 | 2.974 | 213.3 | 0.4 | 13.0 | - |
| 1,000 L supersonic tank (Typhoon) (`typhoontank1000l`) | tank | 5988 | 2.883 | 213.4 | 0.4 | 13.0 | - |
| **52 stores** | | **2124328** | **210.6** | **72156.0** | **97.1** | **864.7** | **18.4** |

By class:

| class | stores | triangles, each | model time s, each | model size kB, each | XML time ms, each | XML size kB, each |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| aam | 24 | 42044 - 121614 | 2.378 - 15.421 | 1508.2 - 4666.2 | 1.4 - 2.1 | 20.9 - 21.1 |
| bomb | 1 | 9980 - 9980 | 2.206 - 2.206 | 423.8 - 423.8 | 47.6 - 47.6 | 19.7 - 19.7 |
| cargo | 2 | 72 - 72 | 0.001 - 0.001 | 8.3 - 8.4 | 0.4 - 0.4 | 8.3 - 8.3 |
| pod | 6 | 5972 - 5990 | 1.726 - 4.503 | 210.1 - 212.4 | 0.4 - 0.5 | 12.9 - 12.9 |
| tank | 19 | 5974 - 6000 | 2.536 - 3.368 | 212.6 - 216.8 | 0.4 - 0.5 | 12.9 - 13.0 |

Stages run: geometry, aero, mass, propulsion, build, model, verify, fly, report.
