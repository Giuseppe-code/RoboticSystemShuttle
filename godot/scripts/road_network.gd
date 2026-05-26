extends Node3D
class_name RoadNetwork

@export_file("*.json") var network_file := "res://data/parco_gandhi_catania_road_network.json"
@export var show_debug_lines := true

var roads: Array = []


func _ready() -> void:
	var raw_text := FileAccess.get_file_as_string(network_file)
	var payload = JSON.parse_string(raw_text)
	if not (payload is Dictionary):
		push_error("Cannot parse road network: " + network_file)
		return
	roads = payload.get("roads", [])
	if show_debug_lines:
		_draw_drivable_centerlines()


func closest_drivable_point(world_position: Vector3) -> Dictionary:
	var query := to_local(world_position)
	var closest := {}
	var minimum_distance := INF
	for road in roads:
		if not road.get("drivable", false):
			continue
		var points: Array = road.get("points_godot_xyz_m", [])
		for index in range(points.size() - 1):
			var start := _vector(points[index])
			var finish := _vector(points[index + 1])
			var candidate := Geometry3D.get_closest_point_to_segment(query, start, finish)
			var distance := query.distance_to(candidate)
			if distance < minimum_distance:
				minimum_distance = distance
				closest = {
					"road_id": road.get("id", ""),
					"highway": road.get("highway", ""),
					"position": to_global(candidate),
					"segment_index": index,
					"distance_m": distance,
					"width_m": road.get("width_m", 0.0),
					"grade_pct": road.get("grade_segments", [])[index].get("grade_pct", 0.0)
				}
	return closest


func road_by_id(road_id: String) -> Dictionary:
	for road in roads:
		if road.get("id", "") == road_id:
			return road
	return {}


func _draw_drivable_centerlines() -> void:
	var lines := ImmediateMesh.new()
	var debug_material := StandardMaterial3D.new()
	debug_material.albedo_color = Color(1.0, 0.78, 0.08, 1.0)
	debug_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	lines.surface_begin(Mesh.PRIMITIVE_LINES, debug_material)
	for road in roads:
		if not road.get("drivable", false):
			continue
		var points: Array = road.get("points_godot_xyz_m", [])
		for index in range(points.size() - 1):
			lines.surface_add_vertex(_vector(points[index]) + Vector3.UP * 0.12)
			lines.surface_add_vertex(_vector(points[index + 1]) + Vector3.UP * 0.12)
	lines.surface_end()
	var instance := MeshInstance3D.new()
	instance.name = "DrivableCenterlinesDebug"
	instance.mesh = lines
	add_child(instance)


func _vector(value: Array) -> Vector3:
	return Vector3(float(value[0]), float(value[1]), float(value[2]))
