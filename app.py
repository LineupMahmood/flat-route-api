import os
import gzip
import math
import urllib.request
import osmnx as ox
import networkx as nx
from flask import Flask, request, jsonify

app = Flask(__name__)

GRAPHML_PATH = "sf_walk_network_elevation.graphml"
GRAPHML_GZ_URL = "https://github.com/LineupMahmood/flat-route-api/releases/download/v1.0/sf_walk_network_elevation.graphml.gz"

if not os.path.exists(GRAPHML_PATH):
    print("Graph file not found. Downloading...")
    gz_path = GRAPHML_PATH + ".gz"
    urllib.request.urlretrieve(GRAPHML_GZ_URL, gz_path)
    print("Download complete. Decompressing...")
    with gzip.open(gz_path, 'rb') as f_in:
        with open(GRAPHML_PATH, 'wb') as f_out:
            f_out.write(f_in.read())
    os.remove(gz_path)
    print("Decompression complete.")

print("Loading elevation network...")
G = ox.load_graphml(filepath=GRAPHML_PATH)

for u, v, k, data in G.edges(keys=True, data=True):
    grade = float(data.get("grade_abs", 0))
    length = float(data.get("length", 0))
    # Exponential penalty — a 10% grade is 7x worse than 5% grade
    # This forces the algorithm onto genuinely flat streets
    data["impedance_high"] = length * (1 + 200 * grade ** 2)
    data["impedance_max"]  = length * (1 + 500 * grade ** 2)

print("Network ready. Server starting...")

def analyze_route(route):
    total_gain = 0
    total_length = 0
    grades = []
    for i in range(len(route) - 1):
        u, v = route[i], route[i+1]
        edge_data = G.get_edge_data(u, v)
        edge = edge_data[0] if edge_data else {}
        length = float(edge.get("length", 0))
        grade = float(edge.get("grade", 0))
        grade_abs = abs(float(edge.get("grade_abs", abs(grade))))
        if length * grade > 0:
            total_gain += length * grade
        total_length += length
        if length > 0:
            grades.append(grade_abs)

    max_grade = max(grades) if grades else 0
    avg_grade = sum(grades) / len(grades) if grades else 0
    coords = [{"lat": G.nodes[n]["y"], "lng": G.nodes[n]["x"]} for n in route]
    return {
        "coordinates": coords,
        "distanceInMiles": round(total_length / 1609.34, 2),
        "elevationGainFt": round(total_gain * 3.281, 1),
        "maxGradePct": round(max_grade * 100, 1),
        "avgGradePct": round(avg_grade * 100, 1),
        "_difficulty": avg_grade * 0.7 + max_grade * 0.3
    }

def get_route_via_waypoint(origin, destination, waypoint_node, weight):
    try:
        if waypoint_node in (origin, destination):
            return None
        leg1 = ox.routing.shortest_path(G, origin, waypoint_node, weight=weight)
        leg2 = ox.routing.shortest_path(G, waypoint_node, destination, weight=weight)
        if leg1 and leg2:
            return leg1 + leg2[1:]
    except:
        pass
    return None

def get_flat_waypoint_nodes(origin, destination, radius_factor=3.0):
    """
    General solution: find actually flat nodes from the graph within
    a search radius, then use those as waypoints. Works for any A->B.
    """
    slat = G.nodes[origin]["y"]
    slng = G.nodes[origin]["x"]
    elat = G.nodes[destination]["y"]
    elng = G.nodes[destination]["x"]

    # Calculate direct distance in meters
    direct_dist_m = math.sqrt(
        ((elat - slat) * 111000) ** 2 +
        ((elng - slng) * 111000 * math.cos(math.radians(slat))) ** 2
    )

    # Search radius — larger radius finds more detour options
    search_radius_m = max(direct_dist_m * radius_factor, 500)

    # Center of search area
    center_lat = (slat + elat) / 2
    center_lng = (slng + elng) / 2

    # Find all flat edges within the search radius
    # A flat edge has grade_abs < 0.04 (4%)
    flat_nodes = set()
    for u, v, k, data in G.edges(keys=True, data=True):
        grade_abs = float(data.get("grade_abs", 1.0))
        if grade_abs < 0.04:  # Only very flat edges
            for node in [u, v]:
                node_lat = G.nodes[node]["y"]
                node_lng = G.nodes[node]["x"]
                dist = math.sqrt(
                    ((node_lat - center_lat) * 111000) ** 2 +
                    ((node_lng - center_lng) * 111000 * math.cos(math.radians(center_lat))) ** 2
                )
                if dist <= search_radius_m:
                    flat_nodes.add(node)

    # Remove origin and destination
    flat_nodes.discard(origin)
    flat_nodes.discard(destination)

    if not flat_nodes:
        return []

    # Sample flat nodes spread across the search area
    # Divide area into a grid and pick the flattest node from each cell
    grid_size = 5
    cells = {}
    for node in flat_nodes:
        node_lat = G.nodes[node]["y"]
        node_lng = G.nodes[node]["x"]
        cell_lat = int((node_lat - slat) / ((elat - slat + 0.001) / grid_size))
        cell_lng = int((node_lng - slng) / ((elng - slng + 0.001) / grid_size))
        cell_key = (cell_lat, cell_lng)
        if cell_key not in cells:
            cells[cell_key] = node
        else:
            # Keep the flatter node
            existing = cells[cell_key]
            existing_grade = min(
                float(G.get_edge_data(existing, nb, 0).get("grade_abs", 1.0))
                for nb in G.neighbors(existing)
            ) if list(G.neighbors(existing)) else 1.0
            new_grade = min(
                float(G.get_edge_data(node, nb, 0).get("grade_abs", 1.0))
                for nb in G.neighbors(node)
            ) if list(G.neighbors(node)) else 1.0
            if new_grade < existing_grade:
                cells[cell_key] = node

    sampled = list(cells.values())
    print(f"Found {len(flat_nodes)} flat nodes, sampled {len(sampled)} waypoints")
    return sampled

def has_loop(route):
    """Detect routes that visit the same node twice — waypoint artifacts."""
    seen = set()
    for coord in route["coordinates"]:
        key = (round(coord["lat"], 5), round(coord["lng"], 5))
        if key in seen:
            return True
        seen.add(key)
    return False

def deduplicate_routes(routes):
    # First remove looping routes
    routes = [r for r in routes if not has_loop(r)]
    
    unique = []
    for r in routes:
        coords = r["coordinates"]
        if len(coords) < 2:
            continue
        # Compare using multiple points along the route, not just midpoint
        sample_indices = [len(coords)//4, len(coords)//2, 3*len(coords)//4]
        is_dup = False
        for u in unique:
            u_coords = u["coordinates"]
            matches = 0
            for idx in sample_indices:
                if idx < len(coords) and idx < len(u_coords):
                    dlat = coords[idx]["lat"] - u_coords[idx]["lat"]
                    dlng = coords[idx]["lng"] - u_coords[idx]["lng"]
                    dist_m = math.sqrt(dlat**2 + dlng**2) * 111000
                    if dist_m < 80:
                        matches += 1
            if matches >= 2:
                is_dup = True
                break
        if not is_dup:
            unique.append(r)
    return unique

@app.route("/health", methods=["GET"])
def health():
    return {"status": "ok"}

@app.route("/route", methods=["GET"])
def get_route():
    try:
        start_lat = float(request.args.get("start_lat"))
        start_lng = float(request.args.get("start_lng"))
        end_lat = float(request.args.get("end_lat"))
        end_lng = float(request.args.get("end_lng"))

        origin = ox.distance.nearest_nodes(G, start_lng, start_lat)
        destination = ox.distance.nearest_nodes(G, end_lng, end_lat)

        all_routes = []

        # Base routes
        for weight in ["impedance_high", "impedance_max", "length"]:
            r = ox.routing.shortest_path(G, origin, destination, weight=weight)
            if r:
                all_routes.append(analyze_route(r))

        # Route through actual flat nodes from the graph
        flat_waypoints = get_flat_waypoint_nodes(origin, destination)
        for wp_node in flat_waypoints:
            for weight in ["impedance_high", "impedance_max"]:
                r = get_route_via_waypoint(origin, destination, wp_node, weight)
                if r:
                    all_routes.append(analyze_route(r))

        unique_routes = deduplicate_routes(all_routes)
        unique_routes.sort(key=lambda r: r["_difficulty"])

        if not unique_routes:
            return jsonify({"error": "No routes found"}), 500

        min_dist = min(r["distanceInMiles"] for r in unique_routes)
        filtered = [r for r in unique_routes if r["distanceInMiles"] <= min_dist * 3.0]

        for r in filtered:
            r.pop("_difficulty", None)

        flat = filtered[0]
        short = min(filtered, key=lambda r: r["distanceInMiles"])

        print(f"✅ Returning {len(filtered)} routes, easiest: avg={filtered[0]['avgGradePct']}%, max={filtered[0]['maxGradePct']}%")
        return jsonify({
            "flatRoute": flat,
            "shortRoute": short,
            "allRoutes": filtered[:6]
        })

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
