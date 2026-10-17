"""fsim's internal shared Python contracts (ARCH-07; docs/architecture/design/directory-layout.md, section 4): what a
public facade and the private implementation behind it both need, kept below both. It imports nothing of fsim.

A private implementation package (fsim._simulation, fsim._training, fsim._visualization) never imports a public module
of fsim. A public class, record or function it implements keeps the public module it always had as its owner -
__module__ names that module, which re-exports it - so its qualified name, repr and pickling are unchanged by where its
code lives."""


def public(module):
    """Declare a class or function implemented privately public in `module`, the facade that re-exports it."""
    def declare(obj):
        obj.__module__ = module
        return obj
    return declare


def options(**given):
    """Only what was given: the library keeps its own defaults for the rest."""
    return {k: (int(v) if isinstance(v, bool) else v) for k, v in given.items() if v is not None}
