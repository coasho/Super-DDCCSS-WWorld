"""Explicit opt-in weapon-link actor process host.

Importing the base SDK facade does not initialize generic process isolation.
This module reuses its existing codec/host and adds only the M03 proxy policy.
"""
from .weapon_link import (WeaponLinksV1 as _WeaponLinksV1,
                         RemoteWeaponLinksV1 as _RemoteWeaponLinksV1,
                         weapon_link_calls_v1, _LINK_CALLS_V1)

__all__ = ["WeaponLinkProcessV1"]

# Reuse the established connection/codec/process loop without extending its
# generic callable lists or exposing an Agent handle in the actor's process.
from .agent import Agent as _Agent
from .isolation import AgentProcess as _AgentProcess, _carried

class _BoundWeaponLinkAgentV1(_Agent):
    __slots__=('_links',)

    def __init__(self,agent):
        # Preserve the original controller. Constructing a new native Agent here
        # would change release ownership and is explicitly forbidden.
        for name in _Agent.__slots__:
            setattr(self,name,getattr(agent,name))
        self._links=_WeaponLinksV1(agent)

    def __getattribute__(self,name):
        if name in _LINK_CALLS_V1:
            return getattr(object.__getattribute__(self,'_links'),name)
        return super().__getattribute__(name)

class _WeaponLinkPolicyV1:
    """Actor-side adapter; the user policy receives only the optional proxy."""
    def __init__(self,remote,entry,*args):
        import importlib
        module,separator,name=entry.partition(':')
        if not separator:
            raise ValueError('weapon-link policy entry must be module:Class')
        policy=getattr(importlib.import_module(module),name)
        self._policy=policy(_RemoteWeaponLinksV1(remote._channel),*args)

    def __getattr__(self,name):
        return getattr(self._policy,name)

class WeaponLinkProcessV1(_AgentProcess):
    """Explicit optional isolated weapon-link actor. Publishing/recording is refused.

    The supplied Range World must be the facade's native World. The actor gets
    RemoteWeaponLinksV1 only; generic Agent, activities, World and referee calls
    are absent from the host's allowlist. No generic gateway/schema is changed.
    """
    def __init__(self,world,agent,entry,*args,path=(),python=None,timeout_s=60):
        from .world import World
        if not isinstance(world,World):
            raise TypeError('WeaponLinkProcessV1 requires the Range World')
        if not isinstance(agent,_Agent):
            raise TypeError('WeaponLinkProcessV1 requires an Agent')
        for value in args:
            _carried(value)
        bound=_BoundWeaponLinkAgentV1(agent)
        if world._h.weapon_links_v1(0,agent.vehicle_id)!=bound._links.instance:
            raise ValueError('weapon-link actor and supplied Range World differ')
        super().__init__(world,bound,'fsim.weapon_link_process:_WeaponLinkPolicyV1',entry,*args,
                         path=path,python=python,allow_side_channels=False,timeout_s=timeout_s)

    def _exchange(self,message):
        # Called during the base constructor too, before policy initialization.
        self._agent_calls=weapon_link_calls_v1()
        self._activity_calls={}
        return super()._exchange(message)

    def call(self,method,*args,**kwargs):
        for value in (*args,*kwargs.values()):
            _carried(value)
        return super().call(method,*args,**kwargs)
