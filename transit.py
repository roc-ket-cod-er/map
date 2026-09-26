from helpers.coords import get_coordinates

from heapq import heappop, heappush
from itertools import count
from bisect import bisect_left
from helpers._transit.make_graph import make_graph
from helpers.distance import dist_time
from helpers.print_color import bold, green, red, blue, magenta
from helpers.time_management import time_to_seconds, seconds_to_time
from datetime import datetime
import time

print("building graph...")

data = make_graph()

print(green("successfully built graph\n"))
print("routing...")

graph = data.graph
stops = data.node_positions

def coordify(stopthingy):
    return stopthingy[0:2]

def transit_a_star(graph, start_id, goal_id, safety_buffer=2, start_time=None):
    """
    safety_buffer: minutes of slack you need between arriving at a stop and a trip's
    departure for that connection to count as catchable at all — this REPLACES the old
    flat transfer_penalty. It's no longer "always add N minutes when switching routes";
    instead the actual cost of a transfer is however long you genuinely have to wait for
    the next trip that respects this buffer, looked up from the real schedule.
    """
    if start_time is None:
        start_time = time_to_seconds(datetime.now().strftime("%H:%M:%S"))  # seconds since midnight

    # Queue stores: (f_score, tie_breaker, current_node, current_trip_id, current_edge_data)
    # The tie_breaker is a unique increasing counter — without it, heapq falls back to
    # comparing trip_id (which mixes None and strings) or edge dicts when priorities tie,
    # and neither of those support '<'.
    tie_breaker = count()
    priority_queue = []
    heappush(priority_queue, (0, next(tie_breaker), start_id, None, None))

    # Track lowest g_score (minutes elapsed since start_time) per state: (node_id, trip_id)
    # trip_id is None when you're not currently riding anything (start, or just walked).
    graph_costs = {(start_id, None): 0}

    # Path reconstructor: (node, trip_id) -> (prev_node, prev_trip_id, edge_data)
    came_from = {}

    while priority_queue:
        current_f, _, current_id, current_trip, info = heappop(priority_queue)

        if current_id == goal_id:
            path = []
            curr_state = (current_id, current_trip)

            while curr_state in came_from:
                prev_node, prev_trip, edge_info = came_from[curr_state]
                path.append((curr_state[0], stops[curr_state[0]], edge_info))
                curr_state = (prev_node, prev_trip)

            path.append((start_id, stops[start_id], None))
            return path[::-1], graph_costs[(current_id, current_trip)]

        current_g = graph_costs.get((current_id, current_trip), float('inf'))
        current_arrival_abs = start_time + current_g * 60  # seconds since midnight, "now" for this state

        for neighbor_id, route_options in graph.get(current_id, {}).items():
            for route_key, trips in route_options.items():
                if route_key == "__walk__":
                    # Walking edges: just one entry, always available immediately
                    edge = trips[0]
                    candidates = [(edge.get("distance", 0), None, edge)]

                else:
                    candidates = []
                    continuing_edge = None
                    if current_trip is not None:
                        continuing_edge = next((t for t in trips if t["trip_id"] == current_trip), None)

                    if continuing_edge is not None:
                        # Still riding the exact same scheduled vehicle — no wait, just ride the hop
                        candidates = [(continuing_edge.get("distance", 0), current_trip, continuing_edge)]
                    else:
                        # Boarding a NEW trip on this route (first ride, transfer, or a later run
                        # of the same route number). trips is sorted by departure_time, so the
                        # earliest catchable one is found in O(log n) instead of trying all of
                        # them — a later trip on the same route can never beat the earliest one
                        # that's actually catchable.
                        earliest_catchable = current_arrival_abs
                        if current_trip is not None:
                            earliest_catchable += safety_buffer * 60

                        dep_times = [t["departure_time"] for t in trips]
                        idx = bisect_left(dep_times, earliest_catchable)
                        if idx < len(trips):
                            edge = trips[idx]
                            wait_minutes = (edge["departure_time"] - current_arrival_abs) / 60.0
                            candidates = [(wait_minutes + edge.get("distance", 0), edge["trip_id"], edge)]
                        # else: nothing on this route is catchable today anymore — no candidate

                for cost, trip_id, edge in candidates:
                    tentative_g = current_g + cost
                    neighbor_state = (neighbor_id, trip_id)

                    if tentative_g < graph_costs.get(neighbor_state, float('inf')):
                        graph_costs[neighbor_state] = tentative_g

                        # Heuristic estimation
                        h = dist_time(coordify(stops[neighbor_id]), coordify(stops[goal_id])) / 60.0
                        priority = tentative_g + h

                        heappush(priority_queue, (priority, next(tie_breaker), neighbor_id, trip_id, edge))
                        came_from[neighbor_state] = (current_id, current_trip, edge)

    return None, float('inf')

st = time.monotonic_ns()
route, total_time = transit_a_star(graph, "grt_busses:2088", "go:UN", safety_buffer=2, start_time=53600)
# print(time.monotonic_ns() - st)

# for item in route:
#     print(item)

if route is None:
    print(red("No route found between the given start and destination."))
    raise SystemExit(1)

i = 1
# Print the route
i = 1

total = {
    "stops": 1,
    "time": 0,
    "departure_time": None,
    "arrival_time": None,
    "first stop": None
}


def print_leg(total, stop_name):
    """Print the currently accumulated leg."""

    fs = total["first stop"]

    if fs is None:
        return

    # Transit
    if fs[3]:
        departure = (
            seconds_to_time(total["departure_time"])
            if total["departure_time"] is not None
            else "?"
        )

        if total["arrival_time"] is not None:
            arrival = seconds_to_time(total["arrival_time"])
        else:
            # Fall back to departure + accumulated travel time
            arrival = seconds_to_time(
                total["departure_time"] + total["time"] * 60
            )

        print(blue(
            f"Ride {total['stops']} stops "
            f"({departure} → {arrival}, "
            f"{total['time']:.1f} minutes) "
            f"from \"{fs[2]}\" to \"{stop_name}\" "
            f"via {fs[0][0]}'s route {fs[3]['route']} "
            f"towards {fs[3]['headsign']}"
        ))

    # Walking
    else:
        if total["time"] > 0.1:
            print(green(
                f"Walk {total['time']:.1f} minutes "
                f"from \"{fs[2]}\" to \"{stop_name}\""
            ))
        else:
            print(magenta(
                f"Transfer from \"{fs[2]}\" to \"{stop_name}\""
            ))


while i < len(route):

    # Current stop
    stop, coords, info = route[i - 1]

    stop_agency, stop_id = stop.split(":")
    stop_name = coords[2]
    stop_agency = stop_agency.split("_")

    # Next stop and the edge connecting us to it
    dnext = route[i]

    ns = dnext[0]
    nc = dnext[1]
    ni = dnext[2] or {}

    ns_agency, ns_id = ns.split(":")
    ns_agency = ns_agency.split("_")
    ns_name = nc[2]

    _info = info or {}

    # Is this edge part of the same leg?
    same_leg = (
        stop_agency == ns_agency
        and ni.get("route") == _info.get("route")
    ) or (
        "route" not in ni
        and "route" not in _info
    )

    # Starting a new leg
    if total["first stop"] is None:
        total["first stop"] = [
            stop_agency,
            stop_id,
            stop_name,
            ni.get("route")
        ]

    # Add this edge to the current leg
    total["stops"] += 1
    total["time"] += ni.get("distance", 0)

    # If this is transit, record the scheduled times
    if "departure_time" in ni:

        # First transit edge = departure from the first stop
        if total["departure_time"] is None:
            total["departure_time"] = ni["departure_time"]

        # Last known arrival time
        if ni.get("arrival_time") is not None:
            total["arrival_time"] = ni["arrival_time"]

    # If this edge continues the current leg, keep going
    if same_leg:
        i += 1
        continue

    # Otherwise, this edge starts a different leg.
    # The current leg ends at the current stop.
    print_leg(total, stop_name)

    # Start a fresh leg at the current stop
    total = {
        "stops": 1,
        "time": 0,
        "departure_time": None,
        "arrival_time": None,
        "first stop": [
            stop_agency,
            stop_id,
            stop_name,
            ni.get("route")
        ]
    }
    i += 1


# Print the final leg
if total["first stop"] is not None:

    # The final destination is the last node in the route
    final_stop, final_coords, final_info = route[-1]

    final_name = final_coords[2]

    print_leg(total, final_name)

print(f"\nEstimated Commute Time: {total_time:.1f} minutes")
