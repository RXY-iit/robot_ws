"""Simulation motion budget with independent wall-time and clock-stop guards."""


class MotionClock:
    def __init__(self, timeout, simulation, source_now, wall_now):
        self.timeout = max(.1, timeout)
        self.simulation = simulation
        self.start_source = self.last_source = source_now
        self.start_wall = self.last_advance_wall = wall_now
        self.wall_limit = min(120., max(self.timeout, self.timeout * 3)) if simulation else self.timeout
        self.maximum_wall_limit = max(self.wall_limit, 300.) if simulation else self.timeout
        self.reason = None

    def expired(self, source_now, wall_now):
        if self.simulation:
            elapsed_wall = wall_now - self.start_wall
            elapsed_source = source_now - self.start_source
            if elapsed_wall >= 5 and elapsed_source > 0:
                # Estimate only the simulation budget, never actuator speed.
                # Progress/odometry guards independently stop stalled motion.
                rtf = elapsed_source / elapsed_wall
                self.wall_limit = min(self.maximum_wall_limit,
                                      max(self.wall_limit, self.timeout / max(.1, rtf)))
            if source_now < self.last_source:
                self.reason = "simulation_clock_reversed"
            elif source_now > self.last_source:
                self.last_advance_wall = wall_now
            self.last_source = source_now
            if wall_now - self.last_advance_wall > 3:
                self.reason = "simulation_clock_stopped"
            if source_now - self.start_source >= self.timeout:
                self.reason = "simulation_time_budget_exhausted"
        if wall_now - self.start_wall >= self.wall_limit:
            self.reason = "wall_time_budget_exhausted"
        return self.reason is not None
