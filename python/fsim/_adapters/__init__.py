"""fsim.isolation's private implementation (ARCH-07, P056): the codec of what crosses an isolated actor's local
pipe (codec) and the actor-side proxies with the actor process's loop (actor). fsim.isolation keeps the Range-side
process hosts and re-exports the public names, which keep it as their owner. It imports fsim._contracts only, never
the package root or a public module."""
