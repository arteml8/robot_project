import asyncio
from basic_motion_planner import BasicWaypointPlanner

# Example waypoints
waypoints = [
    (0.5, 0.0, 0.0),  # Move forward 0.5 meters
    (0.5, 0.5, math.pi/2),  # Move to the side and rotate
    (0.0, 0.5, math.pi),  # Move back to left and rotate
]


def main():
	planner = BasicWaypointPlanner(velocity_controller, tracker)
	planner.load_waypoints(waypoints)

	await planner.run()


if __name__ == "__main__":
    main()