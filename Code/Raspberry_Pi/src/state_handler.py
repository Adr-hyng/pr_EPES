class StateHandler:
    def __init__(self, initial_state):
        self.previous_state = initial_state

    def on_state_change(self, current_state):
        """Checks if the state has changed from the previous state."""
        if current_state != self.previous_state:
            self.previous_state = current_state
            return True  # State has changed
        return False  # No change in state
