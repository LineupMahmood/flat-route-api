import os, gzip, math, urllib.request, pickle
import osmnx as ox
import networkx as nx
from flask import Flask, request, jsonify

app = Flask(__name__)

GRAPHML_PATH = "sf_walk_network_elevation_v4.graphml"
GRAPHML_GZ_URL = "https://github.com/LineupMahmood/flat-route-api/releases/download/V4/sf_walk_network_elevation_v4.graphml.gz"

if not os.path.exists(GRAPHML_PATH):
    print("Downloading graph...")
    gz_path = GRAPHML_PATH + ".gz"
    urllib.request.urlretrieve(GRAPHML_GZ_URL, gz_path)
    with gzip.open(gz_path, 'rb') as f_in:
        with open(GRAPHML_PATH, 'wb') as f_out:
            f_out.write(f_in.read())
    os.remove(gz_path)

PICKLE_PATH = "sf_walk_v11.pkl"
print("Loading graph...")
if os.path.exists(PICKLE_PATH):
    with open(PICKLE_PATH, "rb") as f:
        G = pickle.load(f)
    print("Pickle loaded.")
else:
    G = ox.load_graphml(filepath=GRAPHML_PATH)
    with open(PICKLE_PATH, "wb") as f:
        pickle.dump(G, f)
    print("Pickle saved.")

print("Computing edge weights...")
COMFORT_GRADE = 0.02
K = 2000

ARTERIAL_HIGHWAY = {"primary", "trunk", "motorway"}
arterial_nodes = set()
for u, v, data in G.edges(data=True):
    hw = data.get("highway", "")
    if isinstance(hw, list):
        hw = hw[0] if hw else ""
    lanes_raw = data.get("lanes", "0")
    try:
        lanes = int(str(lanes_raw).split(";")[0].strip())
    except:
        lanes = 0
    if hw in ARTERIAL_HIGHWAY or lanes >= 3:
        arterial_nodes.add(u)
        arterial_nodes.add(v)

print(f"Arterial nodes: {len(arterial_nodes)}")

for u, v, k, data in G.edges(keys=True, data=True):
    grade = float(data.get("grade_abs", 0))
    length = float(data.get("length", 0))
    excess = max(0.0, grade - COMFORT_GRADE)
    hw = data.get("highway", "")
    if isinstance(hw, list):
        hw = hw[0] if hw else ""
    lanes_raw = data.get("lanes", "0")
    try:
        edge_lanes = int(str(lanes_raw).split(";")[0].strip())
    except:
        edge_lanes = 0
    is_arterial_edge = hw in ARTERIAL_HIGHWAY or edge_lanes >= 3
    both_arterial = (u in arterial_nodes and v in arterial_nodes)
    arterial_penalty = 2.5 if (is_arterial_edge or both_arterial) else 1.0

    # Heavy penalty for Van Ness BRT — unpleasant for walking
    name = data.get("name", "")
    if isinstance(name, list):
        name = name[0] if name else ""
    if "bus rapid transit" in str(name).lower():
        arterial_penalty = 10.0

    # Penalty for unnamed edges — prefer named walkable streets
    if not name or str(name).strip() == "":
        unnamed_penalty = 8.0
    else:
        unnamed_penalty = 1.0

    data["impedance"] = length * arterial_penalty * unnamed_penalty * (1 + K * excess ** 2)

print("Ready.")
print("Building spatial index...")
NODE_POSITIONS = {n: (data["y"], data["x"]) for n, data in G.nodes(data=True)}
print(f"Spatial index built: {len(NODE_POSITIONS)} nodes")


# ── Helpers ───────────────────────────────────────────────────────────────────

def haversine(a, b):
    dlat = (a[0] - b[0]) * 111000
    dlng = (a[1] - b[1]) * 111000 * math.cos(math.radians(a[0]))
    return math.sqrt(dlat**2 + dlng**2)


def get_subgraph(start_lat, start_lng, end_lat, end_lng, budget_factor=1.25):
    crow_m = haversine((start_lat, start_lng), (end_lat, end_lng))
    # Semi-major axis of the ellipse
    a = (crow_m * budget_factor) / 2
    # Center of the ellipse
    center_lat = (start_lat + end_lat) / 2
    center_lng = (start_lng + end_lng) / 2
    # Distance from center to each focus
    c = crow_m / 2
    # Semi-minor axis
    b = math.sqrt(max(a**2 - c**2, 1))

    # Angle of the line between start and end
    dlat = (end_lat - start_lat) * 111000
    dlng = (end_lng - start_lng) * 111000 * math.cos(math.radians(center_lat))
    angle = math.atan2(dlng, dlat)

    def inside_ellipse(lat, lng):
        # Distance from this point to both foci
        d1 = haversine((lat, lng), (start_lat, start_lng))
        d2 = haversine((lat, lng), (end_lat, end_lng))
        return (d1 + d2) <= (crow_m * budget_factor)

    nodes = [n for n, (lat, lng) in NODE_POSITIONS.items()
             if inside_ellipse(lat, lng)]
    return G.subgraph(nodes).copy()


def distance_budget(baseline_miles, crow_miles):
    """
    Scale the flat-route distance tolerance based on trip length.
    Short trips get more flexibility, long trips floor at 25%.
    """
    extra = max(0.0, (1.0 - crow_miles) * 0.25)
    return baseline_miles * (1.25 + extra)


def analyze_route(path, graph):
    total_length = 0
    total_gain = 0
    grades = []
    coords = []
    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        ed = graph.get_edge_data(u, v)
        if ed:
            edge = min(ed.values(), key=lambda d: float(d.get("grade_abs", 99)))
            length = float(edge.get("length", 0))
            grade_abs = float(edge.get("grade_abs", 0))
            grade = float(edge.get("grade", 0))
            total_length += length
            if length * grade > 0:
                total_gain += length * grade
            if length > 0:
                grades.append(grade_abs)
    for node in path:
        coords.append({
            "lat": graph.nodes[node]["y"],
            "lng": graph.nodes[node]["x"]
        })
    avg_grade = sum(grades) / len(grades) if grades else 0
    max_grade = max(grades) if grades else 0
    distance_miles = round(total_length / 1609.34, 2)
    return {
        "coordinates": coords,
        "distanceInMiles": distance_miles,
        "elevationGainFt": round(total_gain * 3.281, 1),
        "avgGradePct": round(avg_grade * 100, 1),
        "maxGradePct": round(max_grade * 100, 1),
    }


# ── Endpoints ─────────────────────────────────────────────────────────────────
@app.route("/route_streets")
def route_streets():
    try:
        start_lat = float(request.args.get("start_lat"))
        start_lng = float(request.args.get("start_lng"))
        end_lat   = float(request.args.get("end_lat"))
        end_lng   = float(request.args.get("end_lng"))

        SG = get_subgraph(start_lat, start_lng, end_lat, end_lng)
        origin      = ox.distance.nearest_nodes(SG, start_lng, start_lat)
        destination = ox.distance.nearest_nodes(SG, end_lng, end_lat)

        flat_path = nx.dijkstra_path(SG, origin, destination, weight="impedance")

        streets = []
        for i in range(len(flat_path) - 1):
            u, v = flat_path[i], flat_path[i+1]
            ed = SG.get_edge_data(u, v)
            if ed:
                edge = list(ed.values())[0]
                name = edge.get("name", "unnamed")
                if isinstance(name, list):
                    name = name[0]
                streets.append(name)

        from collections import Counter
        street_counts = Counter(streets)
        return jsonify({
            "streets_in_order": streets,
            "street_summary": dict(street_counts.most_common(10))
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500
@app.route("/health")
def health():
    return {"status": "ok", "version": "v11-distance-constrained"}


@app.route("/route")
def get_route():
    try:
        start_lat = float(request.args.get("start_lat"))
        start_lng = float(request.args.get("start_lng"))
        end_lat   = float(request.args.get("end_lat"))
        end_lng   = float(request.args.get("end_lng"))

        crow_miles = haversine(
            (start_lat, start_lng),
            (end_lat, end_lng)
        ) / 1609.34

        print(f"Trip: ({start_lat},{start_lng}) → ({end_lat},{end_lng}), crow={crow_miles:.2f}mi")

        # ── Extract local subgraph ─────────────────────────────────────────
        SG = get_subgraph(start_lat, start_lng, end_lat, end_lng)
        print(f"Subgraph: {SG.number_of_nodes()} nodes, {SG.number_of_edges()} edges")

        origin      = ox.distance.nearest_nodes(SG, start_lng, start_lat)
        destination = ox.distance.nearest_nodes(SG, end_lng,   end_lat)

        # ── Step 1: Shortest route (baseline) ─────────────────────────────
        short_path = nx.dijkstra_path(SG, origin, destination, weight="length")
        if not short_path:
            return jsonify({"error": "No route found"}), 500

        short_stats = analyze_route(short_path, SG)
        baseline_miles = short_stats["distanceInMiles"]
        budget_miles   = distance_budget(baseline_miles, crow_miles)

        print(f"Short route: {baseline_miles}mi, avg={short_stats['avgGradePct']}%")
        print(f"Flat budget: {budget_miles:.2f}mi")

        # ── Step 2: Flattest route ─────────────────────────────────────────
        flat_path = nx.dijkstra_path(SG, origin, destination, weight="impedance")
        if not flat_path:
            return jsonify({
                "singleRoute": short_stats,
                "message": "Only one route found for this trip.",
            })

        flat_stats = analyze_route(flat_path, SG)
        flat_miles = flat_stats["distanceInMiles"]

        print(f"Flat route: {flat_miles}mi, avg={flat_stats['avgGradePct']}%")

        # ── Step 3: Are they the same route? ──────────────────────────────
        if abs(flat_miles - baseline_miles) <= 0.05:
            print("Routes are effectively identical — returning single route")
            return jsonify({
                "singleRoute": short_stats,
                "message": "The shortest and flattest routes are the same for this trip.",
            })

        # ── Step 4: Is flat route within budget? ──────────────────────────
        grade_saved = round(short_stats["avgGradePct"] - flat_stats["avgGradePct"], 1)
        distance_added = round(flat_miles - baseline_miles, 2)

        if flat_miles <= budget_miles:
            print(f"Flat route within budget — returning both")
            return jsonify({
                "shortRoute": short_stats,
                "flatRoute":  flat_stats,
                "message": f"Flat route adds {distance_added}mi but reduces average grade by {grade_saved}%.",
            })
        else:
            print(f"Flat route too long ({flat_miles:.2f}mi > budget {budget_miles:.2f}mi) — returning short only")
            return jsonify({
                "singleRoute": short_stats,
                "message": "No flatter route found within a reasonable distance for this trip.",
            })

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
