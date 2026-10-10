# Flight stability through the Vehicle Interface

What `fsim testbench --headless stability.` last found, a row an aircraft and load (the suite:
`testbench/fsim_testbench/suites/stability.py`; each row's numbers: `<aircraft>_<load>.json` beside this file).

| aircraft | load | flown | kg | kg as spawned | checks passed | failed |
|---|---|---|---|---|---|---|
| a10c | full | 2026-10-10 | 24041 | 17231 | 54 of 54 | - |
| b52h | full | 2026-10-10 | 221350 | 140390 | 53 of 54 | takeoff: steady after its launch (its height swings 86 m, from two minutes after it) |
| c130j | full | 2026-10-10 | 67731 | 55394 | 54 of 54 | - |
| c172 | full | 2026-10-10 | 1089 | 1089 | 54 of 54 | - |
| c17a | full | 2026-10-10 | 276822 | 235996 | 54 of 54 | - |
| cf2 | full | 2026-10-10 | 0.027 | 0.027 | 37 of 37 | - |
| e3g | full | 2026-10-10 | 149897 | 149895 | 54 of 54 | - |
| e7a | full | 2026-10-10 | 68699 | 60341 | 54 of 54 | - |
| ea18g | full | 2026-10-10 | 26349 | 21532 | 54 of 54 | - |
| ec130h | full | 2026-10-10 | 67932 | 59605 | 54 of 54 | - |
| f15c | full | 2026-10-10 | 26524 | 19149 | 54 of 54 | - |
| f16c | full | 2026-10-10 | 16704 | 12094 | 52 of 54 | turn at 1.5 x its floor: within the bank its speed leaves (70 deg at the most; its speed leaves 60 (its limit 60), past that for 2.3 s)<br>turn at 1.3 x its floor: within the bank its speed leaves (61 deg at the most; its speed leaves 51 (its limit 60), past that for 2.7 s) |
| f22a | full | 2026-10-10 | 37462 | 28147 | 53 of 54 | turn at 1.5 x its floor: within the bank its speed leaves (69 deg at the most; its speed leaves 60 (its limit 60), past that for 0.7 s) |
| f35a | full | 2026-10-10 | 22405 | 21758 | 54 of 54 | - |
| fa18c | full | 2026-10-10 | 20306 | 15989 | 54 of 54 | - |
| gripen | full | 2026-10-10 | 13092 | 9359 | 50 of 54 | orbit right: on its circle (412 m off, of 2176 m radius)<br>orbit right: coordinated (68.8 deg of sideslip)<br>orbit left: on its circle (932 m off, of 2160 m radius)<br>orbit left: coordinated (68.3 deg of sideslip) |
| h6k | full | 2026-10-10 | 73599 | 59198 | 54 of 54 | - |
| iris | full | 2026-10-10 | 1.5 | 1.5 | 37 of 37 | - |
| j10a | full | 2026-10-10 | 19049 | 14859 | 54 of 54 | - |
| j20a | full | 2026-10-10 | 31017 | 29998 | 54 of 54 | - |
| kc135r | full | 2026-10-10 | 145097 | 99737 | 54 of 54 | - |
| kc46a | full | 2026-10-10 | 178972 | 178970 | 54 of 54 | - |
| mig29a | full | 2026-10-10 | 16911 | 14669 | 54 of 54 | - |
| mirage2000 | full | 2026-10-10 | 15919 | 10884 | 53 of 54 | takeoff: steady after its launch (its height swings 29 m, from two minutes after it) |
| rafale | full | 2026-10-10 | 19822 | 14709 | 54 of 54 | - |
| rc135w | full | 2026-10-10 | 134716 | 110901 | 54 of 54 | - |
| rq4b | full | 2026-10-10 | 14628 | 10496 | 54 of 54 | - |
| skua | full | 2026-10-10 | 23.0 | 23.0 | 54 of 54 | - |
| su25 | full | 2026-10-10 | 14674 | 13108 | 54 of 54 | - |
| su27s | full | 2026-10-10 | 22535 | 21818 | 54 of 54 | - |
| su57 | full | 2026-10-10 | 32759 | 28398 | 54 of 54 | - |
| typhoon | full | 2026-10-10 | 19697 | 16175 | 54 of 54 | - |
| u2s | full | 2026-10-10 | 16947 | 12587 | 53 of 54 | turn at its floor: turned, its height kept (4 m lost, 16 deg of bank) |
| uh1h | full | 2026-10-10 | 2793 | 2793 | 37 of 37 | - |
| uh60 | full | 2026-10-10 | 7439 | 7439 | 37 of 37 | - |
