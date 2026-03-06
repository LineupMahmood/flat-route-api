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
    data["impedance_low"]    = length * (1 + 5   * grade ** 2)
    data["impedance_medium"] = length * (1 + 20  * grade ** 2)
    data["impedance_high"]   = length * (1 + 50  * grade ** 2)
    data["impedance_max"]    = length * (1 + 100 * grade ** 2)

print("Network ready. Server starting...")

def route_to_coords(route):
    total_gain = 0
    total_length = 0
    for i in range(len(route) - 1):
        u, v = route[i], route[i+1]
        edge_data = G.get_edge_data(u, v)
        edge = edge_data[0] if edge_data else {}
        length = float(edge.get("length", 0))
        grade = float(edge.get("grade", 0))
        rise = length * grade
        if rise > 0:
            total_gain += rise
        total_length += length
    coords = []
    for node in route:
        node_data = G.nodes[node]
        coords.append({"lat": node_data["y"], "lng": node_data["x"]})
    return {
        "coordinates": coords,
        "distanceInMiles": round(total_length / 1609.34, 2),
        "elevationGainFt": round(total_gain * 3.281, 1)
    }

def get_route_via_waypoint(origin, destination, waypoint_node, weight):
    try:
        if waypoint_node == origin or waypoint_node == destination:
            return None
        leg1 = ox.routing.shortest_path(G, origin, waypoint_node, weight=weight)
        leg2 = ox.routing.shortest_path(G, waypoint_node, destination, weight=weight)
        if leg1 and leg2:
            return leg1 + leg2[1:]
    except:
        pass
    return None

def generate_waypoint_nodes(origin, destination):
    slat, slng = G.nodes[origin]["y"], G.nodes[origin]["x"]
    elat, elng = G.nodes[destination]["y"], G.nodes[destination]["x"]

    lat_diff = elat - slat
    lng_diff = elng - slng
    dist = math.sqrt(lat_diff**2 + lng_diff**2)
    if dist == 0:
        return []

    perp_lat = -lng_diff / dist
    perp_lng = lat_diff / dist

    waypoints = []

    # Standard L-shaped corners
    waypoints.append((slat, elng))
    waypoints.append((elat, slng))

    # Extended corners — go PAST the destination to find routes like Octavia
    # These force the algorithm to explore paths that detour beyond the bounding box
    for extend in [1.5, 2.0, 2.5]:
        extended_lat = slat + lat_diff * extend
        extended_lng = slng + lng_diff * extend
        waypoints.append((slat, extended_lng))   # same start lat, extended dest lng
        waypoints.append((extended_lat, slng))   # extended lat, same start lng
        waypoints.append((extended_lat, elng))   # extended lat, dest lng
        waypoints.append((elat, extended_lng))   # dest lat, extended lng

    # Perpendicular sweeps at multiple fractions along the path
    base_offset = dist * 0.5
    for fraction in [0.25, 0.5, 0.75]:
        base_lat = slat + lat_diff * fraction
        base_lng = slng + lng_diff * fraction
        for offset in [base_offset, base_offset*2, -base_offset, -base_offset*2]:
            waypoints.append((base_lat + perp_lat * offset, base_lng + perp_lng * offset))

    nodes = []
    for lat, lng in waypoints:
        try:
            node = ox.distance.nearest_nodes(G, lng, lat)
            if node not in nodes:
                nodes.append(node)
        except:
            pass
    return nodes

def deduplicate_routes(routes):
    unique = []
    for r in routes:
        coords = r["coordinates"]
        if len(coords) < 2:
            continue
        mid = coords[len(coords)//2]
        is_dup = False
        for u in unique:
            u_coords = u["coordinates"]
            u_mid = u_coords[len(u_coords)//2]
            dlat = mid["lat"] - u_mid["lat"]
            dlng = mid["lng"] - u_mid["lng"]
            dist_m = math.sqrt(dlat**2 + dlng**2) * 111000
            if dist_m < 40:
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

        # Base routes with all penalty levels
        for weight in ["impedance_low", "impedance_medium", "impedance_high", "impedance_max", "length"]:
            r = ox.routing.shortest_path(G, origin, destination, weight=weight)
            if r:
                all_routes.append(route_to_coords(r))

        # Waypoint routes
        waypoint_nodes = generate_waypoint_nodes(origin, destination)
        for wp_node in waypoint_nodes:
            for weight in ["impedance_high", "impedance_max"]:
                r = get_route_via_waypoint(origin, destination, wp_node, weight)
                if r:
                    all_routes.append(route_to_coords(r))

        unique_routes = deduplicate_routes(all_routes)
        unique_routes.sort(key=lambda r: r["elevationGainFt"])

        if len(unique_routes) < 1:
            return jsonify({"error": "No routes found"}), 500

        # Allow routes up to 3x shortest distance if they have lower elevation
        min_dist = min(r["distanceInMiles"] for r in unique_routes)
        min_gain = unique_routes[0]["elevationGainFt"]
        filtered = [r for r in unique_routes
                    if r["distanceInMiles"] <= min_dist * 3.0
                    or r["elevationGainFt"] <= min_gain * 0.6]

        flat = filtered[0]
        short = min(filtered, key=lambda r: r["distanceInMiles"])

        print(f"✅ Returning {len(filtered)} routes, flattest: {flat['elevationGainFt']}ft")
        return jsonify({
            "flatRoute": flat,
            "shortRoute": short,
            "allRoutes": filtered[:6]
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
