"""TEMPORARY: a stand-in for OMS (the mission systems), so the testbench's tactics scenarios can fly realistic
engagements before OMS exists. Everything here is a test algorithm, notional and simple, to be replaced by OMS:

- frame.py    a flat local frame over the arena, the standard atmosphere
- data.py     what each fighter carries: its notional RCS, radar, IRST, missiles (public estimates, rounded)
- missile.py  a notional missile: boost and coast with drag, proportional navigation, loft, datalink, seeker, notch
- sensors.py  radar (range from RCS to the 1/4 power, gimbal), IRST, RWR spike and launch warning, missile warning
- pilot.py    a load-factor pilot on the acceleration level: a heading, a height and a speed at a g limit
- agent.py    the decision rules: picture, search, commit, shoot, support, defend, abort, re-commit, egress

None of it is in the simulation: the aircraft are JSBSim's, everything else is computed here, step by step."""
