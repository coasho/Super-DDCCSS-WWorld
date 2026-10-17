"""A small behaviour-tree executive for the composition example: four node kinds, nothing else. Not a BT library - an
upper-layer runtime brings its own. Its execution state (each node's last status, the ticks it ran) is what a new
generation must clear."""

SUCCESS, FAILURE, RUNNING = "success", "failure", "running"


class Node:
    def __init__(self, name):
        self.name = name


class Condition(Node):
    def __init__(self, name, test):
        super().__init__(name)
        self.test = test

    def tick(self, bt, blackboard):
        return SUCCESS if self.test(blackboard) else FAILURE


class Action(Node):
    def __init__(self, name, act):
        super().__init__(name)
        self.act = act

    def tick(self, bt, blackboard):
        return self.act(blackboard)


class Sequence(Node):
    """Its children in order, until one does not succeed."""

    def __init__(self, name, *children):
        super().__init__(name)
        self.children = children

    def tick(self, bt, blackboard):
        for child in self.children:
            status = bt.run(child, blackboard)
            if status != SUCCESS:
                return status
        return SUCCESS


class Selector(Node):
    """Its children in order, until one does not fail."""

    def __init__(self, name, *children):
        super().__init__(name)
        self.children = children

    def tick(self, bt, blackboard):
        for child in self.children:
            status = bt.run(child, blackboard)
            if status != FAILURE:
                return status
        return FAILURE


class Executive:
    """Ticks one tree; remembers each node's last status and how often it ran."""

    def __init__(self, root):
        self.root = root
        self.status, self.runs = {}, {}

    def reset(self):
        self.status, self.runs = {}, {}

    def run(self, node, blackboard):
        status = node.tick(self, blackboard)
        self.status[node.name] = status
        self.runs[node.name] = self.runs.get(node.name, 0) + 1
        return status

    def tick(self, blackboard):
        return self.run(self.root, blackboard)

    def state(self):
        return {"status": dict(sorted(self.status.items())), "runs": dict(sorted(self.runs.items()))}
