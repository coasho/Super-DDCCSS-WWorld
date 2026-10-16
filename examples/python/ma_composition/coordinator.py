"""The example's reference coordinator (docs/sdk/ma-composition.md, section 4): deterministic and deliberately small.

A proposal names the resources it would hold. A configured owner of a resource wins it; otherwise the higher priority;
otherwise the lower component id, then the earlier proposal. A resource a component's live activity holds is that
component's until the activity ends. An update or cancel addresses only an activity its own component started. The
coordinator never overrides the SDK: what it selects is only submitted, and the SDK still answers - and may refuse -
every request."""

ADDRESSING = {("flight", "update"), ("flight", "cancel"), ("sensor", "cancel")}


class Coordinator:
    def __init__(self, owners=None):
        self.owners = dict(owners or {})  # resource -> component id

    def arbitrate(self, proposals, held, activity_owners):
        """proposals: [(component_id, index, Proposal)]; held: {resource: component id of the live activity holding it};
        activity_owners: {activity id: component id}. -> [(component_id, index, proposal, selected, reason)], in the
        order decided."""
        def rank(item):
            cid, index, p = item
            owned = all(self.owners.get(r) == cid for r in p.resources) and any(r in self.owners for r in p.resources)
            return (0 if owned else 1, -p.priority, cid, index)

        decisions, taken = [], {}
        for cid, index, p in sorted(proposals, key=rank):
            reason = None
            if (p.kind, p.operation) in ADDRESSING:
                owner = activity_owners.get(p.parameters.get("activity_id"))
                if owner != cid:
                    reason = "not_activity_owner:%s" % owner
            for r in p.resources:
                if reason:
                    break
                if self.owners.get(r, cid) != cid:
                    reason = "owned_by:%d" % self.owners[r]
                elif held.get(r, cid) != cid:
                    reason = "held_by:%d" % held[r]
                elif r in taken:
                    reason = "taken_by:%d" % taken[r]
            if reason is None:
                for r in p.resources:
                    taken[r] = cid
            decisions.append((cid, index, p, reason is None, reason or "selected"))
        return decisions
