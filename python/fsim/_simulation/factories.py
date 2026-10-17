"""The agent and referee backends the World wrapper hands out (ARCH-07): World.create_agent and World.referee build them
through these factories, which fsim.agent registers when it loads, so fsim.world never imports fsim.agent - not even
inside a function. The package root loads fsim.agent with fsim.world, before any World exists."""

_agent = None
_referee = None


def register(agent, referee):
    """fsim.agent's Agent and Referee classes: agent(world, vehicle, name) and referee(world)."""
    global _agent, _referee
    _agent, _referee = agent, referee


def _registered(factory, what):
    if factory is None:
        raise RuntimeError("fsim: %s needs fsim.agent, which is not loaded" % what)
    return factory


def agent(world, vehicle, name):
    return _registered(_agent, "World.create_agent")(world, vehicle, name)


def referee(world):
    return _registered(_referee, "World.referee")(world)
