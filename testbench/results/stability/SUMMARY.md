# Flight stability through the Vehicle Interface

What `fsim testbench --headless stability.` last found, a row an aircraft and load (the suite:
`testbench/fsim_testbench/suites/stability.py`; each row's numbers: `<aircraft>_<load>.json` beside this file).

| aircraft | load | flown | kg | kg as spawned | checks passed | failed |
|---|---|---|---|---|---|---|
| a10c | full | 2026-10-11 | 17909 | 17231 | 54 of 54 | - |
| b52h | full | 2026-10-11 | 221350 | 140390 | 53 of 54 | takeoff: steady after its launch (its height swings 88 m, from two minutes after it) |
| c130j | full | 2026-10-11 | 67731 | 55394 | 54 of 54 | - |
| c172 | full | 2026-10-11 | 1089 | 1089 | 54 of 54 | - |
| c17a | full | 2026-10-11 | 253962 | 235996 | 54 of 54 | - |
| cf2 | full | 2026-10-11 | 0.027 | 0.027 | 37 of 37 | - |
| e3g | full | 2026-10-11 | 149897 | 149895 | 54 of 54 | - |
| e7a | full | 2026-10-11 | 68699 | 60341 | 54 of 54 | - |
| ea18g | full | 2026-10-11 | 26349 | 21532 | 54 of 54 | - |
| ec130h | full | 2026-10-11 | 67932 | 59605 | 54 of 54 | - |
| f15c | full | 2026-10-11 | 26524 | 19149 | 54 of 54 | - |
| f16c | full | 2026-10-11 | 16704 | 12094 | 52 of 54 | turn at 1.5 x its floor: within the bank its speed leaves (71 deg at the most; its speed leaves 60 (its limit 60), past that for 2.3 s)<br>turn at 1.3 x its floor: within the bank its speed leaves (61 deg at the most; its speed leaves 51 (its limit 60), past that for 2.7 s) |
| f22a | full | 2026-10-11 | 37462 | 28147 | 52 of 54 | heading +90: within its bank limit (68 deg, its limit 60)<br>turn at 1.5 x its floor: within the bank its speed leaves (69 deg at the most; its speed leaves 60 (its limit 60), past that for 0.8 s) |
| f35a | full | 2026-10-11 | 22405 | 21758 | 54 of 54 | - |
| fa18c | full | 2026-10-11 | 20306 | 15989 | 54 of 54 | - |
| gripen | full | 2026-10-11 | 13092 | 9359 | 54 of 54 | - |
| h6k | full | 2026-10-11 | 84399 | 59198 | 54 of 54 | - |
| iris | full | 2026-10-11 | 1.5 | 1.5 | 37 of 37 | - |
| j10a | full | 2026-10-11 | 19049 | 14859 | 54 of 54 | - |
| j20a | full | 2026-10-11 | 31017 | 29998 | 54 of 54 | - |
| kc135r | full | 2026-10-11 | 145097 | 99737 | 54 of 54 | - |
| kc46a | full | 2026-10-11 | 178972 | 178970 | 54 of 54 | - |
| mig29a | full | 2026-10-11 | 16911 | 14669 | 54 of 54 | - |
| mirage2000 | full | 2026-10-11 | 15919 | 10884 | 53 of 54 | takeoff: steady after its launch (its height swings 35 m, from two minutes after it) |
| rafale | full | 2026-10-11 | 19822 | 14709 | 54 of 54 | - |
| rc135w | full | 2026-10-11 | 134716 | 110901 | 54 of 54 | - |
| rq4b | full | 2026-10-11 | 14628 | 10496 | 54 of 54 | - |
| skua | full | 2026-10-11 | 23.0 | 23.0 | 54 of 54 | - |
| su25 | full | 2026-10-11 | 14674 | 13108 | 54 of 54 | - |
| su27s | full | 2026-10-11 | 22535 | 21818 | 54 of 54 | - |
| su57 | full | 2026-10-11 | 32759 | 28398 | 54 of 54 | - |
| typhoon | full | 2026-10-11 | 19697 | 16175 | 53 of 54 | takeoff: steady after its launch (its height swings 24 m, from two minutes after it) |
| u2s | full | 2026-10-11 | 16947 | 12587 | 53 of 54 | turn at its floor: turned, its height kept (4 m lost, 16 deg of bank) |
| uh1h | full | 2026-10-11 | 2793 | 2793 | 37 of 37 | - |
| uh60 | full | 2026-10-11 | 7439 | 7439 | 37 of 37 | - |
