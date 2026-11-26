"""Physical button banks from Roki_2_Soccer/main_killable_all.py."""

from .head_menu import Item, test


def start(label, strategy, **args):
    return Item(label, op="game.start", args=dict(strategy=strategy, **args))


def groups(label, left=(), middle=(), right=()):
    return (label, {183: left, 185: middle, 186: right})


def layout():
    run = groups("Run test",
                 (test("Side step left", "run_test", mode="side_left"), test("Rotation left", "rotation_test")),
                 tuple(test(label, "run_test", mode=mode) for label, mode in
                       (("Short run", "short"), ("Long run", "long"), ("Spot run", "spot"), ("Run backwards", "backwards"))),
                 (test("Side step right", "run_test", mode="side_right"), test("Rotation right", "rotation_test")))
    def jump(direction):
        return test("Jump " + direction.replace("_", " "), "jump_test", direction=direction)
    jumps = groups("Jump test", (jump("left"),), tuple(jump(d) for d in ("forward", "backward", "on_spot")), (jump("right"),))
    def entry(position):
        return tuple(start(label, "simple_football", entry=position, delay_seconds=delay)
                     for label, delay in ((position.title(), 0), (position.title() + " later", 10)))
    football = groups("Simple football", entry("left"), entry("center"), entry("right"))
    goals = groups("Own goal", (Item("Own yellow goal", op="menu.own_goal", args={"colour": "yellow"}),),
                   (), (Item("Own blue goal", op="menu.own_goal", args={"colour": "blue"}),))
    kick = groups("Kick test", middle=(test("Regular", "kick_test", mode="regular"),
                  start("Ball approach and kick", "ball_kick_test")))
    return groups("Choose role", (football,), (goals,), (run, jumps, kick, test("Get up test", "get_up_test")))



def label(choice):
    return choice[0] if isinstance(choice, tuple) else choice.label
