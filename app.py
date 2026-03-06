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
    data["impedance_high"] = length * (1 + 50  * grade ** 2)
    data["impedance_max"]  = length * (1 + 100 * grade ** 2)

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
        if length * grade > 0:
            total_gain += length * grade
        total_length += length
    coords = [{"lat": G.nodes[n]["y"], "lng": G.nodes[n]["x"]} for n in route]
    return {
        "coordinates": coords,
        "distanceInMiles": round(total_length / 1609.34, 2),
        "elevationGainFt": round(total_gain * 3.281, 1)
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

    waypoints = [
        # L-shaped corners
        (slat, elng),
        (elat, slng),
        # Extended corners past destination
        (slat + lat_diff * 1.5, slng),
        (slat, slng + lng_diff * 1.5),
        # Midpoint perpendicular sweeps
        ((slat+elat)/2 + perp_lat*dist*0.5, (slng+elng)/2 + perp_lng*dist*0.5),
        ((slat+elat)/2 - perp_lat*dist*0.5, (slng+elng)/2 - perp_lng*dist*0.5),
    ]

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
        is_dup = any(
            math.sqrt((mid["lat"]-u["coordinates"][len(u["coordinates"])//2]["lat"])**2 +
                      (mid["lng"]-u["coordinates"][len(u["coordinates"])//2]["lng"])**2) * 111000 < 40
            for u in unique
        )
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

        for weight in ["impedance_high", "impedance_max", "length"]:
            r = ox.routing.shortest_path(G, origin, destination, weight=weight)
            if r:
                all_routes.append(route_to_coords(r))

        for wp_node in generate_waypoint_nodes(origin, destination):
            for weight in ["impedance_high", "impedance_max"]:
                r = get_route_via_waypoint(origin, destination, wp_node, weight)
                if r:
                    all_routes.append(route_to_coords(r))

        unique_routes = deduplicate_routes(all_routes)
        unique_routes.sort(key=lambda r: r["elevationGainFt"])

        if not unique_routes:
            return jsonify({"error": "No routes found"}), 500

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
