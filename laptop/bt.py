"""
A tiny behavior tree library.

Every node has a tick(bb) method that returns SUCCESS, FAILURE or RUNNING.
`bb` is the blackboard: one shared object holding the latest sensor readings
and the motor command the tree wants.

    Selector  - tries children in order, returns the first one that doesn't fail
                (this is what gives us priorities)
    Sequence  - runs children in order, stops at the first one that doesn't succeed
                (we use it as "if Condition then Action")
    Condition - checks something on the blackboard: SUCCESS if true, FAILURE if false
    Action    - does something (sets bb.drive, requests a sound, ...)
"""

SUCCESS = "SUCCESS"
FAILURE = "FAILURE"
RUNNING = "RUNNING"


class Node:
    def __init__(self, name):
        self.name = name

    def tick(self, bb):
        raise NotImplementedError


class Selector(Node):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status != FAILURE:
                return status
        return FAILURE


class Sequence(Node):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self, bb):
        for child in self.children:
            status = child.tick(bb)
            if status != SUCCESS:
                return status
        return SUCCESS


class Condition(Node):
    def __init__(self, name, check):
        super().__init__(name)
        self.check = check  # function(bb) -> bool

    def tick(self, bb):
        return SUCCESS if self.check(bb) else FAILURE


class Action(Node):
    """Pass a function(bb) -> status, or subclass and override run(bb)."""

    def __init__(self, name, func=None):
        super().__init__(name)
        self.func = func

    def run(self, bb):
        return self.func(bb)

    def tick(self, bb):
        bb.active_node = self.name  # so the main loop can print what's running
        return self.run(bb)
