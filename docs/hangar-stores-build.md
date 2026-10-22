Batch result: PASS. Child exit codes: 463lpallet=0, a10tank600gal=0, aim120c=0, aim120d=0, aim7m=0, aim9m=0, aim9x=0, anaaq28=0, analq131=0, analq184=0, analq99=0, anasq228=0, asraam=0, cdsa22=0, cj10a=0, damocles=0, f22tank600gal=0, fpu11=0, fpu8=0, gripentank1100l=0, irist=0, j10tank1600l=0, j10tank800l=0, magic2=0, meteor=0, micaem=0, micair=0, mk82=0, pl10=0, pl12=0, pl15=0, pl8=0, ptb1150=0, ptb1500=0, ptb2000=0, ptb800=0, r27er=0, r27et=0, r27r=0, r27t=0, r37m=0, r60m=0, r73=0, r77m=0, rpl522=0, rpl541=0, rpl701=0, rpl751=0, super530d=0, tank300gal=0, tank370gal=0, tank610gal=0, typhoontank1000l=0.

Cached stage values below are historical unless produced by this run.

# Stores: what the pipeline built

Every store on the cards in `assets/stores`, through hangar's stages ([hangar.md](hangar.md#stores)) - written by
`python -m hangar stores --force --jobs 1 --record docs/hangar-stores-build.md`. Each row is one store's last run: its model for
the viewer (`aircraft/<name>/<name>.glb`) and its JSBSim flight model (`aircraft/stores/<name>/<name>.xml`; a
missile's motor and nozzle files beside it, `Engines/`).

- **Run:** 2026-10-22 01:21:23 to 2026-10-22 01:28:07, hangar 0.1.0, Windows-11-10.0.26300-SP0, 16 logical processors, one store at a time.
- **Made again:** every output - each model meshed anew, each flight model's files removed and written (`--force`).
- **Model time:** the mesher's, the paint's and the file's writing (a cargo store's box is drawn, not meshed).
- **XML time:** the build-up from the card and the design, the text and the file's writing - the engine files with it.

| store | class | triangles | model time s | model size kB | XML time ms | XML size kB | motor files kB |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 463L pallet, loaded (`463lpallet`) | cargo | 72 | 0.001 | 8.3 | 0.5 | 8.6 | - |
| 600 US gal external tank (A-10) (`a10tank600gal`) | tank | 5996 | 2.908 | 215.1 | 0.5 | 13.3 | - |
| AIM-120C-7 AMRAAM (`aim120c`) | aam | 83092 | 4.011 | 2534.6 | 10.0 | 67.4 | 0.8 |
| AIM-120D AMRAAM (`aim120d`) | aam | 83092 | 4.019 | 2534.8 | 9.8 | 67.4 | 0.8 |
| AIM-7M Sparrow (`aim7m`) | aam | 102230 | 4.271 | 3466.0 | 9.7 | 67.4 | 0.8 |
| AIM-9M Sidewinder (`aim9m`) | aam | 87164 | 3.796 | 2762.8 | 10.0 | 65.6 | 0.7 |
| AIM-9X Block I Sidewinder (`aim9x`) | aam | 42044 | 3.651 | 1512.4 | 10.2 | 67.4 | 0.7 |
| AN/AAQ-28 LITENING (`anaaq28`) | pod | 5986 | 2.664 | 211.9 | 0.5 | 13.2 | - |
| AN/ALQ-131(V) (`analq131`) | pod | 5976 | 4.131 | 212.1 | 0.5 | 13.2 | - |
| AN/ALQ-184(V) (`analq184`) | pod | 5972 | 4.489 | 210.1 | 0.5 | 13.2 | - |
| AN/ALQ-99 (`analq99`) | pod | 5990 | 3.781 | 212.4 | 0.5 | 13.2 | - |
| AN/ASQ-228 ATFLIR (`anasq228`) | pod | 5984 | 1.714 | 210.4 | 0.5 | 13.2 | - |
| AIM-132 ASRAAM (`asraam`) | aam | 48826 | 4.534 | 1676.9 | 9.4 | 67.4 | 0.8 |
| CDS bundle (A-22) (`cdsa22`) | cargo | 72 | 0.001 | 8.4 | 0.5 | 8.6 | - |
| CJ-10A (`cj10a`) | bomb | 9986 | 3.819 | 524.9 | 49.7 | 20.0 | - |
| Damocles (`damocles`) | pod | 5980 | 2.845 | 211.7 | 0.6 | 13.2 | - |
| 600 US gal external tank (F-22) (`f22tank600gal`) | tank | 5996 | 2.920 | 215.5 | 0.6 | 13.3 | - |
| FPU-11/A 480 US gal tank (`fpu11`) | tank | 5990 | 3.357 | 216.8 | 0.5 | 13.3 | - |
| FPU-8/A 330 US gal tank (`fpu8`) | tank | 5988 | 2.932 | 215.1 | 0.5 | 13.2 | - |
| 1,100 L external tank (Gripen) (`gripentank1100l`) | tank | 5992 | 2.969 | 215.8 | 0.5 | 13.3 | - |
| IRIS-T (`irist`) | aam | 48110 | 4.859 | 1734.0 | 10.3 | 67.4 | 0.7 |
| 1,600 L external tank (J-10) (`j10tank1600l`) | tank | 5988 | 2.877 | 213.4 | 0.5 | 13.3 | - |
| 800 L external tank (J-10) (`j10tank800l`) | tank | 6000 | 3.008 | 215.1 | 0.6 | 13.2 | - |
| R.550 Magic 2 (`magic2`) | aam | 74120 | 4.238 | 2652.4 | 9.8 | 65.4 | 0.7 |
| Meteor (`meteor`) | aam | 43312 | 4.299 | 1508.3 | 9.6 | 67.4 | 0.8 |
| MICA EM (`micaem`) | aam | 58218 | 5.991 | 1916.3 | 9.8 | 67.4 | 0.7 |
| MICA IR (`micair`) | aam | 58330 | 6.137 | 1925.4 | 10.0 | 67.4 | 0.7 |
| Mk 82 (`mk82`) | bomb | 9980 | 2.208 | 423.8 | 48.9 | 20.0 | - |
| PL-10 (`pl10`) | aam | 56118 | 5.350 | 1850.6 | 10.0 | 67.3 | 0.7 |
| PL-12 (`pl12`) | aam | 92526 | 4.926 | 2858.7 | 9.9 | 67.3 | 0.8 |
| PL-15 (`pl15`) | aam | 48264 | 3.579 | 1584.2 | 9.9 | 67.4 | 1.0 |
| PL-8 (`pl8`) | aam | 114090 | 8.161 | 4326.4 | 16.1 | 94.7 | 0.7 |
| PTB-1150 (`ptb1150`) | tank | 5992 | 2.917 | 214.8 | 0.5 | 13.2 | - |
| PTB-1500 (`ptb1500`) | tank | 5992 | 2.922 | 215.6 | 0.5 | 13.2 | - |
| PTB-2000 (`ptb2000`) | tank | 5974 | 2.900 | 213.1 | 0.5 | 13.3 | - |
| PTB-800 (`ptb800`) | tank | 6000 | 2.903 | 214.9 | 0.7 | 13.2 | - |
| R-27ER (`r27er`) | aam | 109066 | 6.121 | 3769.5 | 15.5 | 95.9 | 0.8 |
| R-27ET (`r27et`) | aam | 107396 | 5.705 | 3699.9 | 16.0 | 95.9 | 0.8 |
| R-27R (`r27r`) | aam | 107510 | 6.045 | 3865.6 | 17.1 | 96.2 | 0.7 |
| R-27T (`r27t`) | aam | 114120 | 6.850 | 4105.0 | 15.8 | 96.1 | 0.7 |
| R-37M (`r37m`) | aam | 92614 | 6.314 | 2739.2 | 9.9 | 67.3 | 1.0 |
| R-60M (`r60m`) | aam | 75664 | 2.323 | 2262.5 | 9.9 | 65.5 | 0.7 |
| R-73 (`r73`) | aam | 105128 | 5.225 | 3590.7 | 16.2 | 94.6 | 0.7 |
| R-77M (`r77m`) | aam | 121614 | 15.254 | 4666.3 | 10.1 | 67.4 | 1.0 |
| RPL-522 (`rpl522`) | tank | 5996 | 2.534 | 215.4 | 0.5 | 13.2 | - |
| RPL-541/542 (`rpl541`) | tank | 5996 | 3.014 | 214.3 | 0.5 | 13.3 | - |
| RPL-701 (`rpl701`) | tank | 5992 | 2.862 | 215.3 | 0.5 | 13.2 | - |
| RPL-751 (`rpl751`) | tank | 5974 | 2.897 | 213.8 | 0.6 | 13.3 | - |
| Super 530D (`super530d`) | aam | 91840 | 5.871 | 2829.1 | 10.1 | 67.4 | 0.8 |
| 300 US gal centerline tank (F-16) (`tank300gal`) | tank | 5992 | 2.951 | 212.6 | 0.5 | 13.3 | - |
| 370 US gal external tank (F-16) (`tank370gal`) | tank | 5990 | 2.795 | 214.2 | 0.5 | 13.3 | - |
| 610 US gal external tank (F-15) (`tank610gal`) | tank | 5992 | 2.959 | 213.3 | 0.5 | 13.3 | - |
| 1,000 L supersonic tank (Typhoon) (`typhoontank1000l`) | tank | 5988 | 2.910 | 213.4 | 0.5 | 13.3 | - |
| **53 stores** | | **2134314** | **212.7** | **72683.0** | **388.0** | **2168.9** | **18.8** |

By class:

| class | stores | triangles, each | model time s, each | model size kB, each | XML time ms, each | XML size kB, each |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| aam | 24 | 42044 - 121614 | 2.323 - 15.254 | 1508.3 - 4666.3 | 9.4 - 17.1 | 65.4 - 96.2 |
| bomb | 2 | 9980 - 9986 | 2.208 - 3.819 | 423.8 - 524.9 | 48.9 - 49.7 | 20.0 - 20.0 |
| cargo | 2 | 72 - 72 | 0.001 - 0.001 | 8.3 - 8.4 | 0.5 - 0.5 | 8.6 - 8.6 |
| pod | 6 | 5972 - 5990 | 1.714 - 4.489 | 210.1 - 212.4 | 0.5 - 0.6 | 13.2 - 13.2 |
| tank | 19 | 5974 - 6000 | 2.534 - 3.357 | 212.6 - 216.8 | 0.5 - 0.7 | 13.2 - 13.3 |

Stages run: geometry, aero, mass, propulsion, build, model, verify, fly, report.
