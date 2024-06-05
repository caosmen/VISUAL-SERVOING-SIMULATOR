from statemachine import StateMachine, State


class VP6242Machine(StateMachine):
    """
    Class representing the state machine of the VP6242 robot.

    States:
        - IDLE: The robot is idle.
        - SEARCH: The robot is searching for the target.
        - TRACK: The robot is tracking the target.
        - END: The robot has reached the target.

    Transitions:
        - IDLE -> SEARCH: The robot starts searching for the target.
        - IDLE -> TRACK: The robot starts tracking the target.
        - SEARCH -> TRACK: The robot has found the target and starts tracking it.
        - TRACK -> SEARCH: The robot has lost the target and starts searching for it.
        - TRACK -> END: The robot has reached the target.
    """

    idle = State('IDLE', initial=True)
    search = State('SEARCH')
    track = State('TRACK')
    end = State('END', final=True)

    start_search = idle.to(search)
    start_track = idle.to(track)
    found_target = search.to(track)
    lost_target = track.to(search)
    reached_target = track.to(end)
